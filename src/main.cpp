#include <Arduino.h>
#include <WiFi.h>

// Pin Definitions
const int hallPin = 4;
const int mapPin = 34;
const int ignitionPin = 5;
const int handbrakePin = 13;

// Variables for RPM calculation
const uint8_t PULSES_PER_REV = 35;
const unsigned long UPDATE_INTERVAL_MS = 200;
volatile unsigned long pulseCount = 0;
float rpm = 0.0f;

// Timing map: [MAP (kPa)][RPM] -> Ignition advance (degrees BTDC)
int timingMap[5][12] = {
  {20, 22, 24, 26, 28, 30, 32, 34, 35, 35, 35, 35}, // MAP 20-40 kPa (high vacuum, idle/light load)
  {18, 20, 22, 24, 26, 28, 30, 32, 33, 34, 34, 34}, // MAP 40-60 kPa
  {15, 17, 19, 21, 23, 25, 27, 29, 31, 32, 32, 32}, // MAP 60-80 kPa
  {12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30, 30}, // MAP 80-100 kPa
  {10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30}  // MAP 100-120 kPa (low vacuum, WOT)
};

// Wi-Fi settings
const char* WIFI_SSID = "ESP32_RPM";
const char* WIFI_PASSWORD = "esp32pass";
WiFiServer server(80);
WiFiClient client;

// Debug blink function
void blinkLED(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(2, HIGH);
    delay(duration);
    digitalWrite(2, LOW);
    delay(duration);
  }
}

// Interrupt handler for Hall sensor (RPM calculation)
void IRAM_ATTR onHallPulse() {
  pulseCount++;
}

// Read MAP sensor (MPX5700AP, 0-700 kPa, 0.2-4.7V output) with averaging
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
  // Uncomment to print raw ADC values over Wi-Fi
  // if (client && client.connected()) {
  //   char debugMsg[32];
  //   snprintf(debugMsg, sizeof(debugMsg), "Raw ADC: %d Volt: %.2f\n", raw, voltage);
  //   client.println(debugMsg);
  // }
  if (raw < 300) {
    return 100.0; // Default to 100 kPa when sensor not connected
  }
  return kPa;
}

// Calculate ignition advance based on RPM and MAP
int getAdvance(int rpm, float map) {
  int rpmIdx = constrain((rpm - 500) / 500, 0, 11);
  int mapIdx = constrain((int)(map - 20) / 20, 0, 4);
  int baseAdvance = timingMap[mapIdx][rpmIdx];
  return constrain(baseAdvance, 5, 35);
}

void setup() {
  // Step 1: Blink 1 time - Start of setup
  pinMode(2, OUTPUT);
  blinkLED(1, 200);

  // Initialize Wi-Fi as Access Point
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  // Step 2: Blink 2 times - After Wi-Fi AP setup
  blinkLED(2, 200);

  // Start TCP server
  server.begin();
  blinkLED(3, 200);

  // Pin setup
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(mapPin, INPUT); // ADC pin, no pull-up needed
  pinMode(ignitionPin, OUTPUT);
  pinMode(handbrakePin, INPUT_PULLUP);

  // Step 4: Blink 4 times - After pin setup
  blinkLED(4, 200);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, RISING);

  // Step 5: Blink 5 times - Setup complete
  blinkLED(5, 200);
}

unsigned long nextSpark = 0;

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastUpdate < UPDATE_INTERVAL_MS) {
    // handle new client connections
    if (!client || !client.connected()) {
      WiFiClient newClient = server.available();
      if (newClient) {
        client = newClient;
      }
    }
    return;
  }
  lastUpdate = nowMs;
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();
  rpm = (pulses * (60000.0f / UPDATE_INTERVAL_MS)) / PULSES_PER_REV;
  bool cutIgnition = (digitalRead(handbrakePin) == LOW);
  float map = readMAP();
  int advance = getAdvance(static_cast<int>(rpm), map);
  if (!cutIgnition && micros() >= nextSpark && rpm > 0.0f) {
    digitalWrite(ignitionPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(ignitionPin, LOW);
    unsigned long period = static_cast<unsigned long>(60000000.0f / rpm);
    unsigned long advanceTime = (period * advance) / 360;
    nextSpark = micros() + period - advanceTime;
  }
  Serial.printf("RPM: %.2f\n", rpm);
  if (client && client.connected()) {
    client.printf("%.2f\n", rpm);
  }
  delay(10);
}