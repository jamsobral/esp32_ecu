#include <Arduino.h>
#include <WiFi.h>

// ======= Hardware Pins =======
#define DISTRIBUTOR_PIN     4    // Inductive pickup input
#define MAP_SENSOR_PIN      34   // MAP sensor ADC input
#define IGNITION_COIL_PIN   5    // MOSFET gate for ignition coil
#define HANDBRAKE_PIN       13   // Handbrake switch input
#define LED_PIN             2    // On-board status LED

// ======= WiFi & TCP Server (Soft AP) =======
const char* apSsid = "ESP32_Ignition";
const char* apPassword = "e3012345678";
WiFiServer server(80);
WiFiClient client;

// ======= Ignition Map Params =======
const int RPM_STEPS = 14;
const int MAP_STEPS = 5;
const unsigned long rpmBreakpoints[RPM_STEPS] = {500,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000};
const float mapBreakpoints[MAP_STEPS] = {10.0,30.0,50.0,70.0,100.0};
int ignitionMap[RPM_STEPS][MAP_STEPS] = {
  {5,7,9,11,13},   {6,8,10,12,14},  {8,10,12,14,16}, {10,12,14,16,18},
  {12,14,16,18,20},{14,16,18,20,22},{16,18,20,22,24},{18,20,22,24,26},
  {19,21,23,25,27},{20,22,24,26,28},{21,23,25,27,29},{22,24,26,28,30},
  {23,25,27,29,31},{24,26,28,30,32}
};

// ======= RPM Sensing =======
volatile unsigned long periodUs = 0, lastPulseUs = 0;
volatile bool pulseFlag = false;
int rpmSmoothed = 0;
const int PULSES_PER_REV = 4;   // pulses per engine revolution

void IRAM_ATTR pulseInterrupt() {
  unsigned long now = micros();
  static unsigned long lastInterrupt = 0;
  if (now - lastInterrupt < 15000) return; // debounce 15 ms
  periodUs = now - lastPulseUs;
  lastPulseUs = now;
  pulseFlag = true;
  lastInterrupt = now;
}

// Read MAP sensor with averaging and transfer function
float readMAP() {
  const int samples = 10;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(MAP_SENSOR_PIN);
    delayMicroseconds(50);
  }
  float v = (sum / (float)samples) / 4095.0 * 3.3;      // ADC to voltage
  v = v * 2.0 * 1.5185;                                // divider and correction
  float kPa = ((v / 5.05) - 0.04) / 0.0012858;        // sensor transfer
  return constrain(kPa, 10.0, 120.0);
}

// Linear interpolation helper
float interp(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) return y0;
  return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

// Bilinear lookup for ignition advance
int lookupAdvance(float kPa, int rpm) {
  int i = 1;
  while (i < RPM_STEPS && rpm >= rpmBreakpoints[i]) i++;
  int j = 1;
  while (j < MAP_STEPS && kPa >= mapBreakpoints[j]) j++;
  int i0 = max(0, i - 1), i1 = min(i, RPM_STEPS - 1);
  int j0 = max(0, j - 1), j1 = min(j, MAP_STEPS - 1);
  float a00 = ignitionMap[i0][j0], a10 = ignitionMap[i1][j0];
  float a01 = ignitionMap[i0][j1], a11 = ignitionMap[i1][j1];
  float r0 = rpmBreakpoints[i0], r1 = rpmBreakpoints[i1];
  float m0 = mapBreakpoints[j0], m1 = mapBreakpoints[j1];
  float adv0 = interp(rpm, r0, r1, a00, a10);
  float adv1 = interp(rpm, r0, r1, a01, a11);
  return round(interp(kPa, m0, m1, adv0, adv1));
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Boot blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  Serial.begin(115200);
  pinMode(DISTRIBUTOR_PIN, INPUT);
  pinMode(MAP_SENSOR_PIN, INPUT);
  pinMode(IGNITION_COIL_PIN, OUTPUT);
  pinMode(HANDBRAKE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DISTRIBUTOR_PIN), pulseInterrupt, RISING);
  digitalWrite(IGNITION_COIL_PIN, LOW);

  // Start WiFi AP and server
  WiFi.softAP(apSsid, apPassword);
  server.begin();
  // LED solid to show AP ready
  digitalWrite(LED_PIN, HIGH);
  Serial.printf("AP mode SSID=%s IP=%s", apSsid, WiFi.softAPIP().toString().c_str());
}

void loop() {
  static unsigned long lastLogMs = 0;
  const unsigned long logInterval = 200; // log every 200ms

  // Update RPM
  if (pulseFlag) {
    pulseFlag = false;
    if (periodUs > 5000 && periodUs < 1000000) {
      unsigned long raw = 60000000UL / periodUs;
      int engRpm = raw / PULSES_PER_REV;
      rpmSmoothed = (rpmSmoothed * 3 + engRpm) / 4;
    } else {
      rpmSmoothed = (rpmSmoothed * 3) / 4;
    }
  }
  int rpm = rpmSmoothed;

  // Handle client connect
  if (!client || !client.connected()) {
    WiFiClient nc = server.available();
    if (nc) {
      client = nc;
      // Blink on connect
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // Read sensors & compute advance
  float mapVal = readMAP();
  int advance = lookupAdvance(mapVal, rpm);

  // Fire ignition coil
  digitalWrite(IGNITION_COIL_PIN, HIGH);
  delayMicroseconds(1000);  // 1ms dwell
  digitalWrite(IGNITION_COIL_PIN, LOW);

  // Throttled logging
  unsigned long now = millis();
  if (now - lastLogMs >= logInterval) {
    String msg = "RPM:" + String(rpm) + ",MAP:" + String(mapVal,1) + ",ADV:" + String(advance);
    Serial.println(msg);
    if (client && client.connected()) {
      client.println(msg);
    }
    lastLogMs = now;
  }
}