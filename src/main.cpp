#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h> // For watchdog timer

// Pin Definitions
const int hallPin = 4;
const int mapPin = 34;
const int ignitionPin = 5;
const int handbrakePin = 13;

// Variables for RPM calculation
volatile unsigned long lastPulse = 0;
volatile unsigned long period = 0;
volatile int rpm = 0;

// Timing map: [MAP (kPa)][RPM] -> Ignition advance (degrees BTDC)
int timingMap[5][12] = {
  {20, 22, 24, 26, 28, 30, 32, 34, 35, 35, 35, 35}, // MAP 20-40 kPa (high vacuum, idle/light load)
  {18, 20, 22, 24, 26, 28, 30, 32, 33, 34, 34, 34}, // MAP 40-60 kPa
  {15, 17, 19, 21, 23, 25, 27, 29, 31, 32, 32, 32}, // MAP 60-80 kPa
  {12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30, 30}, // MAP 80-100 kPa
  {10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30}  // MAP 100-120 kPa (low vacuum, WOT)
};

// Wi-Fi settings
const char* ssid = "ESP32_Ignition";
WiFiServer server(80);
WiFiClient client;
bool deviceConnected = false;

// Debug blink function
void blinkLED(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(2, HIGH);
    delay(duration);
    digitalWrite(2, LOW);
    delay(duration);
  }
}

// Interrupt handler for Hall sensor (RPM calculation) with debouncing
void IRAM_ATTR pulseInterrupt() {
  static unsigned long lastInterrupt = 0;
  unsigned long now = micros();
  if (now - lastInterrupt < 1000) { // Ignore interrupts within 1ms
    return;
  }
  lastInterrupt = now;
  period = now - lastPulse;
  lastPulse = now;
}

// Read MAP sensor (MPX5700AP, 0-700 kPa, 0.2-4.7V output) with averaging and sanity check
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
  if (deviceConnected && client.connected()) {
    char debugMsg[32];
    snprintf(debugMsg, sizeof(debugMsg), "Raw ADC: %d Volt: %.2f\n", raw, voltage);
    client.println(debugMsg);
  }
  if (raw < 300 || kPa < 20 || kPa > 120) { // Reasonable MAP range
    return 100.0; // Default to 100 kPa if out of range
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
  // Initialize watchdog timer with 5-second timeout
  esp_task_wdt_init(5, true); // 5 seconds, panic on timeout
  esp_task_wdt_add(NULL); // Add current task to watchdog

  // Step 1: Blink 1 time - Start of setup
  pinMode(2, OUTPUT);
  blinkLED(1, 200);

  // Initialize Wi-Fi as Access Point
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();

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
  attachInterrupt(digitalPinToInterrupt(hallPin), pulseInterrupt, FALLING);

  // Step 5: Blink 5 times - Setup complete
  blinkLED(5, 200);
}

unsigned long nextSpark = 0;

void loop() {
  // Reset watchdog timer
  esp_task_wdt_reset();

  // Check temperature (ESP32 internal sensor, may need calibration)
  int temp = (int)temperatureRead(); // Returns temperature in Celsius
  if (temp > 85) { // 85°C threshold
    digitalWrite(ignitionPin, LOW); // Disable ignition
    while (true) {
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100); // Blink rapidly to indicate overheat
    }
  }

  // Check for client connection
  static bool lastConnectedState = false;
  if (!client.connected()) {
    client = server.available(); // Listen for incoming clients
    deviceConnected = client.connected();
  } else {
    deviceConnected = true;
  }

  // Handle connection/disconnection events
  if (deviceConnected && !lastConnectedState) {
    blinkLED(3, 50); // Blink 3 times on connect
  } else if (!deviceConnected && lastConnectedState) {
    blinkLED(5, 500); // Blink 5 times on disconnect
    client.stop();
  }
  lastConnectedState = deviceConnected;

  // Read handbrake state
  bool cutIgnition = (digitalRead(handbrakePin) == LOW);

  // Read MAP sensor
  float map = readMAP();

  // Update RPM from interrupt data with sanity check
  if (period > 0) {
    rpm = 60000000 / period;
    if (rpm > 7000) { // Cap RPM to prevent unrealistic values
      rpm = 0;
    }
  } else {
    rpm = 0;
  }

  // Calculate ignition advance
  int advance = getAdvance(rpm, map);

  // Fire ignition coil if handbrake isn't active
  if (!cutIgnition && micros() >= nextSpark && rpm > 0) {
    digitalWrite(ignitionPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(ignitionPin, LOW);
    unsigned long period = 60000000 / rpm;
    unsigned long advanceTime = (period * advance) / 360;
    nextSpark = micros() + period - advanceTime;
  }

  // Send data every 100ms if connected
  static unsigned long lastUpdate = 0;
  if (deviceConnected && client.connected() && millis() - lastUpdate >= 100) {
    // Blink LED fast (50ms) with each message
    digitalWrite(2, HIGH);
    delay(50);
    digitalWrite(2, LOW);
    delay(50);

    // Use char buffer to send data
    char message[64];
    snprintf(message, sizeof(message), "RPM: %d MAP: %.1f Adv: %d HB: %s\n",
             rpm, map, advance, cutIgnition ? "ON" : "OFF");
    client.println(message);

    lastUpdate = millis();
  }

  // Blink LED slow (500ms) when idle or not connected
  if (!deviceConnected || (millis() - lastUpdate >= 100)) {
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
  }

  delay(10);
}