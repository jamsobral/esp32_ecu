#include <Arduino.h>
#include <WiFi.h>

// ======= Hardware Pins =======
#define DISTRIBUTOR_PIN    4    // Inductive pickup input (via MAX9926)
#define MAP_SENSOR_PIN     34   // MAP sensor ADC input
#define IGNITION_COIL_PIN  5    // MOSFET gate for ignition coil
#define HANDBRAKE_PIN      13   // Handbrake switch input
#define LED_PIN            2    // On‑board status LED

// ======= WiFi & TCP Server (Soft AP) =======
const char* apSsid     = "ESP32_Ignition";
const char* apPassword = "e3012345678";
WiFiServer server(80);
WiFiClient client;
bool deviceConnected = false;

// ======= Linear Ignition Map (RPM 2000‑7000, 500 RPM steps) =======
const int MIN_RPM   = 2000;
const int RPM_STEP  = 500;
const int MAP_ZONES = 6;
const int RPM_ZONES = 11;  // from 2000 to 7000 inclusive
// MAP zones: 20,40,60,80,100,120 kPa
int timingMap[MAP_ZONES][RPM_ZONES] = {
  {22, 24, 26, 30, 34, 38, 42, 42, 41, 40, 38},  // MAP 20 kPa
  {20, 22, 24, 28, 32, 36, 38, 40, 39, 38, 36},  // MAP 40 kPa
  {18, 20, 22, 26, 30, 33, 35, 36, 36, 35, 34},  // MAP 60 kPa
  {16, 18, 20, 23, 26, 28, 30, 31, 31, 30, 29},  // MAP 80 kPa
  {14, 16, 18, 21, 24, 26, 27, 28, 28, 27, 26},  // MAP 100 kPa
  {12, 14, 16, 18, 20, 22, 23, 24, 25, 25, 26}   // MAP 120 kPa
};

// ======= RPM Sensing Params & Rev‑Hang =======
volatile unsigned long lastPulseMicros     = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile bool sawNewPulse                  = false;
const uint8_t NUM_LOBES                    = 4;      // Distributor lobes
const float   DECEL_ALPHA                  = 0.3f;   // Rev‑hang smoothing
const float   MAX_EXPECTED_RPM             = 9000.0f; // updated ceiling
const int     PULSES_PER_REV               = 4;
int rpmSmoothed = 0;

// ======= ISR =======
void IRAM_ATTR pulseInterrupt() {
  unsigned long now = micros();
  static unsigned long lastInterrupt = 0;
  if (lastPulseMicros) {
    pulseIntervalMicros = now - lastPulseMicros;
    sawNewPulse = true;
  }
  lastPulseMicros = now;
  lastInterrupt    = now;
}

// ======= MAP Sensor =======
float readMAP() {
  const int samples = 10;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(MAP_SENSOR_PIN);
    delayMicroseconds(50);
  }
  float v = (sum / (float)samples) / 4095.0f * 3.3f;
  v = v * 2.0f * 1.5185f;
  float kPa = ((v / 5.05f) - 0.04f) / 0.0012858f;
  return constrain(kPa, 20.0f, 120.0f);
}

// ======= Simple Linear Lookup =======
int getAdvance(int rpm, float kPa) {
  int rpmIdx = constrain((rpm - MIN_RPM) / RPM_STEP, 0, RPM_ZONES - 1);
  int mapIdx = constrain((int)((kPa - 20) / 20), 0, MAP_ZONES - 1);
  return timingMap[mapIdx][rpmIdx];
}

// ======= Setup =======
void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }
  Serial.begin(115200);
  pinMode(DISTRIBUTOR_PIN, INPUT);
  pinMode(MAP_SENSOR_PIN, INPUT);
  pinMode(IGNITION_COIL_PIN, OUTPUT);
  pinMode(HANDBRAKE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DISTRIBUTOR_PIN), pulseInterrupt, RISING);
  digitalWrite(IGNITION_COIL_PIN, LOW);
  WiFi.softAP(apSsid, apPassword);
  server.begin();
  digitalWrite(LED_PIN, HIGH);
}

// ======= Main Loop =======
void loop() {
  // RPM rev‑hang smoothing
  if (sawNewPulse) {
    sawNewPulse = false;
    if (pulseIntervalMicros > 5000 && pulseIntervalMicros < 1000000) {
      float rawRpm = (60000000UL / pulseIntervalMicros) / PULSES_PER_REV;
      if (rawRpm > MAX_EXPECTED_RPM) rawRpm = rpmSmoothed;
      if (rawRpm >= rpmSmoothed) rpmSmoothed = rawRpm;
      else rpmSmoothed = DECEL_ALPHA * rawRpm + (1.0f - DECEL_ALPHA) * rpmSmoothed;
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
      digitalWrite(LED_PIN, LOW); delay(100);
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // Read sensors & compute advance
  float mapVal = readMAP();
  int advance = getAdvance(rpm, mapVal);

  // Fire ignition coil: 1ms dwell
  digitalWrite(IGNITION_COIL_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(IGNITION_COIL_PIN, LOW);

  // Logging every 100ms
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 100) {
    char msg[96];
    int hb = digitalRead(HANDBRAKE_PIN) == LOW;
    snprintf(msg, sizeof(msg), "[%lu] RPM:%d,MAP:%.1f,ADV:%d,HB:%d", millis(), rpm, mapVal, advance, hb);
    Serial.println(msg);
    if (client && client.connected()) client.println(msg);
    lastLog = millis();
  }
}
