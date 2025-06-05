#include <Arduino.h>
#include <WiFi.h>

// === Pin Definitions ===
const int hallPin = 4;
const int mapPin = 34;
const int ignitionPin = 5;
const int handbrakePin = 13;

// === Trigger Wheel Setup (36-1, missing tooth 19 teeth after TDC) ===
const int TEETH_COUNT = 36;
const int DEGREES_PER_TOOTH = 10;               // 360 / 36
const int MISSING_TOOTH_TO_TDC_DEG = 190;       // 19 teeth × 10° each

// === Position Tracking ===
volatile int toothCounter = -1;
volatile unsigned long lastToothTime = 0;
volatile unsigned long lastInterval = 0;
volatile bool gapDetected = false;

// === Tooth Interval Averaging (2 intervals) ===
#define RPM_AVG_TEETH 2
volatile unsigned long toothIntervals[RPM_AVG_TEETH] = {0};
volatile int toothIntervalIdx = 0;

// === RPM and Ignition ===
float rpm = 0;
float mapValue = 100;  // Renamed from "map"
unsigned long nextSparkMicros = 0;

// === Timing Map: [MAP (kPa)][RPM] ===
int timingMap[5][12] = {
  {20, 22, 24, 26, 28, 30, 32, 34, 35, 35, 35, 35},
  {18, 20, 22, 24, 26, 28, 30, 32, 33, 34, 34, 34},
  {15, 17, 19, 21, 23, 25, 27, 29, 31, 32, 32, 32},
  {12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30, 30},
  {10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30}
};

// === Wi-Fi ===
const char* WIFI_SSID = "ESP32_RPM";
const char* WIFI_PASSWORD = "esp32pass";
WiFiServer server(80);
WiFiClient client;

// === Interrupt Handler ===
void IRAM_ATTR onHallPulse() {
  unsigned long now = micros();
  unsigned long interval = now - lastToothTime;

  // Store interval for averaging
  toothIntervals[toothIntervalIdx] = interval;
  toothIntervalIdx = (toothIntervalIdx + 1) % RPM_AVG_TEETH;

  // Detect missing tooth (gap): interval much larger than normal
  if (interval > lastInterval * 1.8) { // Raised threshold for robustness
    toothCounter = 0;
    gapDetected = true;
  } else if (toothCounter >= 0) {
    toothCounter++;
    if (toothCounter >= (TEETH_COUNT - 1)) toothCounter = 0; // wrap around
  }

  lastInterval = interval;
  lastToothTime = now;
}

// === MAP Sensor Reading ===
float readMAP() {
  const int numReadings = 5;
  long total = 0;
  for (int i = 0; i < numReadings; i++) {
    total += analogRead(mapPin);
    delayMicroseconds(100);
  }
  int raw = total / numReadings;
  float voltage = (raw / 4095.0) * 3.3;
  float kPa = (voltage - 0.2) * (700 - 15) / (4.7 - 0.2) + 15;
  return (raw < 300) ? 100.0 : kPa;
}

// === Advance Calculation ===
int getAdvance(int rpmVal, float mapVal) {
  int rpmIdx = constrain((rpmVal - 500) / 500, 0, 11);
  int mapIdx = constrain((int)(mapVal - 20) / 20, 0, 4);
  return constrain(timingMap[mapIdx][rpmIdx], 5, 35);
}

// === RPM Calculation from Averaged Tooth Intervals ===
float calcRPM() {
  noInterrupts();
  unsigned long sum = 0;
  /**
   * Main loop:
   * - Handle new client connections (if any)
   * - Update RPM calculation every 100 ms
   * - Update MAP sensor reading
   * - Run engine position and ignition logic:
   *   - Calculate crankshaft angle from tooth counter
   *   - Get advance angle from timing map
   *   - Trigger ignition if conditions are met
   * - Output debug information to serial monitor and client (if connected)
   * - Sleep for 5 ms
   */
  for (int i = 0; i < RPM_AVG_TEETH; i++) sum += toothIntervals[i];
  interrupts();

  float avgInterval = (float)sum / RPM_AVG_TEETH;
  if (avgInterval == 0) return 0;
  return 60000000.0 / (avgInterval * TEETH_COUNT);
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
  // Always handle new client connections
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) client = newClient;
  }

  // Update RPM calculation every 100 ms
  static unsigned long lastRPMCalc = 0;
  unsigned long now = millis();

  if (now - lastRPMCalc >= 100) {
    rpm = calcRPM();
    mapValue = readMAP();
    lastRPMCalc = now;
  }

  bool cutIgnition = digitalRead(handbrakePin) == LOW;

  // Engine position and ignition logic
  if (gapDetected && toothCounter >= 0 && rpm > 0) {
    int crankAngle = (toothCounter * DEGREES_PER_TOOTH - MISSING_TOOTH_TO_TDC_DEG + 360) % 360;
    int advanceAngle = getAdvance(rpm, mapValue);

    if (!cutIgnition && crankAngle >= (360 - advanceAngle) && micros() >= nextSparkMicros) {
      digitalWrite(ignitionPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(ignitionPin, LOW);

      unsigned long revPeriod = 60000000UL / rpm;
      nextSparkMicros = micros() + revPeriod;
    }
    gapDetected = false;
  }

  // Debug output
  Serial.printf("RPM: %.1f, MAP: %.1f, Adv: %d\n", rpm, mapValue, getAdvance(rpm, mapValue));

  if (client.connected()) {
    client.printf("RPM: %.1f MAP: %.1f Adv: %d\n", rpm, mapValue, getAdvance(rpm, mapValue));
  }

  delay(5);
}
