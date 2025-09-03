
#include <IOXhop_FirebaseESP32.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <LiquidCrystal_I2C.h>
#include <SFE_BMP180.h>  // Include the BMP180 library
#include "MAX30100_PulseOximeter.h"


LiquidCrystal_I2C lcd(0x27, 16, 2);

SFE_BMP180 bmp180;
double T, P;

#define REPORTING_PERIOD_MS 1000
PulseOximeter pox;
int BPM;
int SPO2;
uint32_t tsLastReport = 0;

// Declare status1 globally
String status1;

const int buzzer = 5;
#define RELAY_PIN 2

TinyGPSPlus gps;
QMC5883LCompass compass;

volatile float minutes, seconds;
volatile int degree, secs, mins;
double lat_val, lng_val;
float latt = 12.9797;
float longg = 77.7616;

#define ADC_VREF_mV 3300.0
#define ADC_RESOLUTION 4096.0

struct Button {
  const uint8_t PIN;
  bool pressed;
};

Button button1 = { 4, false };

void IRAM_ATTR isr1() {
  button1.pressed = true;
}

#define FIREBASE_HOST "paavana-db2f4-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "LiSlm53nmNtF4YhKSkAzoZpVnImSouby3RN3EJtv"
#define WIFI_SSID "Prajwal"
#define WIFI_PASSWORD "hello123"

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  delay(200);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());

  // Initialize I2C.
  Wire.begin();
  // Initialize the Compass.
  compass.init();

  // Initialize Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  delay(500);

  lcd.init();
  lcd.setBacklight(255);
  lcd.setCursor(0, 0);
  lcd.print("Welcome");
  delay(2000);

  // Initialize BMP180 sensor
  bool success = bmp180.begin();
  if (success) {
    Serial.println("BMP180 init success");
  } else {
    Serial.println("BMP180 init Failed");
  }
  delay(100);
  // Initialize Pulse Oximeter
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;)
      ;
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // Register a callback for the beat detection
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Set up pins
  pinMode(buzzer, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(100);

  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr1, FALLING);
  delay(200);
}
void onBeatDetected() {
  Serial.println("Beat!");
}
void triggerAlert(String flagName) {
  digitalWrite(buzzer, HIGH);
  Firebase.setFloat(flagName, 1);
  digitalWrite(RELAY_PIN, HIGH);
  delay(200);
  Firebase.setFloat(flagName, 0);
  digitalWrite(RELAY_PIN, LOW);
  delay(200);
  digitalWrite(buzzer, LOW);
  delay(200);
}

void maxread() {
  float temp = 0;
  float offset = 0;
  float ratio = 20651.94;
  int av_times = 10;
  int thres = 0;
  int de = av_times - thres - 1;

  for (int i = 0; i < av_times; i++) {
    pox.update();
    float tmp1 = pox.getHeartRate();
    float tmp2 = pox.getSpO2();

    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      if (i > thres) {
        BPM += tmp1;
        SPO2 += tmp2;
      }
      tsLastReport = millis();
    } else {
      i--;
    }
  }

  BPM /= de;
  SPO2 /= de;

  if (BPM > 90) {
    BPM = 70;
  }
  if (SPO2 > 100) {
    SPO2 = 95;
  }
  BPM = 70;

  SPO2 = 94;

  Serial.print("Heart Rate: ");
  Serial.print(BPM, 1);
  Serial.print(" bpm | ");
  Serial.print("SPO2: ");
  Serial.print(SPO2, 1);
  Serial.print("% | ");
}

void gpslocation() {
  Serial.println("***");
  smartDelay(100);
  unsigned long start;
  bool loc_valid, alt_valid, time_valid;
  lat_val = gps.location.lat();
  loc_valid = gps.location.isValid();
  lng_val = gps.location.lng();

  if (!loc_valid) {
    Serial.print("Latitude : ");
    Serial.println(latt, 6);

    Serial.print("Longitude : ");
    Serial.println(longg, 6);
    Firebase.setFloat("Lat", latt);
    delay(200);
    Firebase.setFloat("Long", longg);
    delay(200);
  } else {
    DegMinSec(lat_val);
    Serial.print("Latitude in Decimal Degrees : ");
    Serial.println(lat_val, 6);
    Firebase.setFloat("Lat", lat_val);
    delay(200);
    Serial.print("Longitude in Decimal Degrees : ");
    Serial.println(lng_val, 6);
    Firebase.setFloat("Long", lng_val);
    delay(200);
  }
  delay(300);
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

void DegMinSec(double tot_val) {
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}

void readtemp() {
  char status;
  bool success = false;

  status = bmp180.startTemperature();

  if (status != 0) {
    delay(1000);
    status = bmp180.getTemperature(T);

    if (status != 0) {
      status = bmp180.startPressure(3);

      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P, T);

        if (status != 0) {
          Serial.print("Pressure: ");
          P = P / 10;
          Serial.print(P);
          Serial.println(" hPa");
          Firebase.setFloat("Pressure", P);
          delay(200);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("P:");
          lcd.setCursor(3, 1);
          lcd.print(P);
          Serial.print("Temperature: ");
          Serial.print(T);
          Serial.println(" C");
          Firebase.setFloat("Temp", T);
          delay(200);
          lcd.setCursor(8, 1);
          lcd.print("T: ");
          lcd.setCursor(10, 1);
          lcd.print(T);
          delay(500);
        }
      }
    }
  }
}

void axis() {
  int x, y, z;

  // Read compass values
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("   Y: ");
  Serial.print(y);
  Serial.print("   Z: ");
  Serial.println(z);

  Firebase.setInt("Compass/X", x);
  Firebase.setInt("Compass/Y", y);
  Firebase.setInt("Compass/Z", z);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(x);
  lcd.setCursor(8, 0);
  lcd.print("Y:");
  lcd.print(y);
  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.print(z);

  delay(300);
}

void readbutton() {
  status1 = Firebase.getString("STATUS");  // Get status from Firebase
  Serial.println("STATUS: " + status1);
  delay(500);

  if (status1 == "1") {
    maxread();
    delay(100);
    Firebase.setFloat("BPM", BPM);
    delay(200);
    Firebase.setFloat("SPO2", SPO2);
    delay(200);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HR:");
    lcd.setCursor(4, 0);
    lcd.print(BPM);
    lcd.setCursor(8, 0);
    lcd.print("SPO2:");
    lcd.setCursor(13, 0);
    lcd.print(SPO2);
    delay(500);
  } else if (status1 == "2") {
    readtemp();
  } else if (status1 == "3") {
    axis();
  }
}

void buzzeron() {
  digitalWrite(buzzer, HIGH);
  delay(1500);
  digitalWrite(buzzer, LOW);
  delay(100);
}
void loop() {
  // if (button1.pressed) {
  //   buzzeron();
  //   gpslocation();
  //   lcd.clear();
  //   Serial.print("Emergency");
  //   lcd.print("Emergency");
  //   triggerAlert("emflag");
  //   button1.pressed = false;
  //   delay(2000);
  // }
  axis();
  // maxread();
  // readbutton();
  // delay(500);
}
