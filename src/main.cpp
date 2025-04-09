#include <Arduino.h>
#include <WiFi.h>

// Pin Definitions
const int hallPin = 4;        // Hall sensor for RPM
const int mapPin = 34;        // GPIO 34 for MAP sensor ADC
const int ignitionPin = 5;    // Ignition coil control
const int handbrakePin = 13;  // Handbrake switch

// Variables for RPM calculation
volatile unsigned long lastPulse = 0;
volatile unsigned long period = 0;
volatile int rpm = 0;

// Timing map: [MAP (kPa)][RPM] -> Ignition advance (degrees BTDC)
int timingMap[5][12] = {
  {20, 22, 24, 26, 28, 30, 32, 34, 35, 35, 35, 35}, // MAP 20-40 kPa
  {18, 20, 22, 24, 26, 28, 30, 32, 33, 34, 34, 34}, // MAP 40-60 kPa
  {15, 17, 19, 21, 23, 25, 27, 29, 31, 32, 32, 32}, // MAP 60-80 kPa
  {12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30, 30}, // MAP 80-100 kPa
  {10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30}  // MAP 100-120 kPa
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
  if (now - lastInterrupt < 1000) { // Debounce: ignore pulses within 1ms
    return;
  }
  lastInterrupt = now;
  period = now - lastPulse;
  lastPulse = now;
}

// Read MAP sensor with ADC correction and exact transfer function
float readMAP() {
  const int numReadings = 10;
  long total = 0;
  for (int i = 0; i < numReadings; i++) {
    total += analogRead(mapPin);
    delayMicroseconds(100);
  }
  int raw = total / numReadings;
  float calculatedVoltage = (raw / 4095.0) * 3.3;         // ESP32 ADC: 0-4095 = 0-3.3V
  float actualVoltage = calculatedVoltage * 1.5185;       // Correction factor (0.41V / 0.27V)
  float sensorVoltage = actualVoltage * 2.0;              // Voltage divider ratio = 0.5, so Vout = 2 * V_GPIO34
  float Vs = 5.05;                                        // Measured supply voltage
  float kPa = ((sensorVoltage / Vs) - 0.04) / 0.0012858;  // Exact transfer function
  if (raw < 300 || kPa < 20 || kPa > 120) {
    return 100.0; // Default to 100 kPa if out of range
  }
  return kPa;
}

// Calculate ignition advance based on RPM and MAP
int getAdvance(int rpm, float map) {
  int rpmIdx = constrain((rpm - 500) / 500, 0, 11);  // RPM 500-6500, steps of 500
  int mapIdx = constrain((int)(map - 20) / 20, 0, 4); // MAP 20-120 kPa, steps of 20
  int baseAdvance = timingMap[mapIdx][rpmIdx];
  return constrain(baseAdvance, 5, 35);              // Limit advance to 5-35° BTDC
}

void setup() {
  // Step 1: Blink 1 time - Start of setup
  pinMode(2, OUTPUT);
  blinkLED(1, 200);

  // Initialize Wi-Fi as Access Point
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();

  // Step 2: Blink 2 times - After Wi-Fi setup
  blinkLED(2, 200);

  // Start TCP server
  server.begin();
  blinkLED(3, 200);

  // Pin setup
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(mapPin, INPUT);         // ADC pin
  pinMode(ignitionPin, OUTPUT);
  pinMode(handbrakePin, INPUT_PULLUP);

  // Step 4: Blink 4 times - After pin setup
  blinkLED(4, 200);

  // Attach interrupt for RPM
  attachInterrupt(digitalPinToInterrupt(hallPin), pulseInterrupt, FALLING);

  // Step 5: Blink 5 times - Setup complete
  blinkLED(5, 200);
}

unsigned long nextSpark = 0;

void loop() {
  // Check ESP32 temperature (basic overheat protection)
  int temp = (int)temperatureRead();
  if (temp > 85) { // 85°C threshold
    digitalWrite(ignitionPin, LOW);
    while (true) {
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100); // Rapid blink to indicate overheat
    }
  }

  // Handle client connection
  static bool lastConnectedState = false;
  if (!client.connected()) {
    client = server.available();
    deviceConnected = client.connected();
  } else {
    deviceConnected = true;
  }

  // Connection status feedback
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

  // Update RPM
  if (period > 0) {
    rpm = 60000000 / period; // Convert period (µs) to RPM
    if (rpm > 7000) {
      rpm = 0; // Cap unrealistic values
    }
  } else {
    rpm = 0;
  }

  // Calculate ignition advance
  int advance = getAdvance(rpm, map);

  // Fire ignition coil if conditions met
  if (!cutIgnition && micros() >= nextSpark && rpm > 0) {
    digitalWrite(ignitionPin, HIGH);
    delayMicroseconds(1000); // 1ms dwell time
    digitalWrite(ignitionPin, LOW);
    unsigned long period_us = 60000000 / rpm;
    unsigned long advanceTime = (period_us * advance) / 360;
    nextSpark = micros() + period_us - advanceTime;
  }

  // Send data to client every 100ms
  static unsigned long lastUpdate = 0;
  if (deviceConnected && client.connected() && millis() - lastUpdate >= 100) {
    digitalWrite(2, HIGH);
    delay(50);
    digitalWrite(2, LOW);
    delay(50);

    char message[64];
    snprintf(message, sizeof(message), "RPM: %d MAP: %.1f Adv: %d HB: %s\n",
             rpm, map, advance, cutIgnition ? "ON" : "OFF");
    client.println(message);

    lastUpdate = millis();
  }

  // Idle blink if no client or no update
  if (!deviceConnected || (millis() - lastUpdate >= 100)) {
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
  }

  delay(10);
}