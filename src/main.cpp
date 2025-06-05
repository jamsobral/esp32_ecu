#include <Arduino.h>
#include <WiFi.h>

// Pin Definitions
const int hallPin = 4;
const int mapPin = 34;
const int ignitionPin = 5;
const int handbrakePin = 13;

// Trigger wheel setup (36-1)
const int TEETH_COUNT = 36;
const int MISSING_TOOTH_ANGLE = 190; // degrees before TDC
const int DEGREES_PER_TOOTH = 360 / TEETH_COUNT;
const int TDC_TOOTH = MISSING_TOOTH_ANGLE / DEGREES_PER_TOOTH;

// Variables
volatile int toothCounter = -1;
volatile unsigned long lastToothTime = 0;
volatile unsigned long toothInterval = 0;
volatile bool gapDetected = false;

// RPM and Ignition
float rpm = 0;
float map = 100;
unsigned long nextSparkMicros = 0;

// Timing map [MAP (kPa)][RPM] -> Ignition advance (degrees BTDC)
int timingMap[5][12] = {
  {20, 22, 24, 26, 28, 30, 32, 34, 35, 35, 35, 35},
  {18, 20, 22, 24, 26, 28, 30, 32, 33, 34, 34, 34},
  {15, 17, 19, 21, 23, 25, 27, 29, 31, 32, 32, 32},
  {12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30, 30},
  {10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30}
};

// Wi-Fi
const char* WIFI_SSID = "ESP32_RPM";
const char* WIFI_PASSWORD = "esp32pass";
WiFiServer server(80);
WiFiClient client;

// Interrupt handler
void IRAM_ATTR onHallPulse() {
  unsigned long now = micros();
  unsigned long interval = now - lastToothTime;

  if (interval > toothInterval * 1.5) { // Detect missing tooth
    toothCounter = 0;
    gapDetected = true;
  } else if (toothCounter >= 0) {
    toothCounter++;
  }

  toothInterval = interval;
  lastToothTime = now;
}

float readMAP() {
  int raw = analogRead(mapPin);
  float voltage = (raw / 4095.0) * 3.3;
  float kPa = (voltage - 0.2) * (700 - 15) / (4.7 - 0.2) + 15;
  return (raw < 300) ? 100.0 : kPa;
}

int getAdvance(int rpm, float map) {
  int rpmIdx = constrain((rpm - 500) / 500, 0, 11);
  int mapIdx = constrain((int)(map - 20) / 20, 0, 4);
  return constrain(timingMap[mapIdx][rpmIdx], 5, 35);
}

void setup() {
  Serial.begin(115200);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(ignitionPin, OUTPUT);
  pinMode(handbrakePin, INPUT_PULLUP);

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  server.begin();

  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, RISING);
}

void loop() {
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) client = newClient;
  }

  static unsigned long lastRPMCalc = 0;
  unsigned long now = millis();

  if (now - lastRPMCalc >= 100) {
    noInterrupts();
    unsigned long intervalCopy = toothInterval;
    interrupts();

    rpm = (intervalCopy > 0) ? 60000000.0 / (intervalCopy * TEETH_COUNT) : 0;
    map = readMAP();

    lastRPMCalc = now;
  }

  bool cutIgnition = digitalRead(handbrakePin) == LOW;

  if (gapDetected && toothCounter >= 0 && rpm > 0) {
    int crankAngle = (toothCounter * DEGREES_PER_TOOTH - MISSING_TOOTH_TO_TDC_DEG + 360) % 360;
    int advanceAngle = getAdvance(rpm, map);

    if (!cutIgnition && crankAngle >= (360 - advanceAngle) && micros() >= nextSparkMicros) {
        digitalWrite(ignitionPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(ignitionPin, LOW);

        unsigned long revPeriod = 60000000UL / rpm;
        nextSparkMicros = micros() + revPeriod;
    }
    gapDetected = false;
  }

  Serial.printf("RPM: %.1f, MAP: %.1f, Adv: %d\n", rpm, map, getAdvance(rpm, map));

  if (client.connected()) {
    client.printf("RPM: %.1f MAP: %.1f Adv: %d\n", rpm, map, getAdvance(rpm, map));
  }

  delay(5);
}
