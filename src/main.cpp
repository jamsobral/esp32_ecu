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

// ======= Ignition Map (MAP × RPM) =======
const int MIN_RPM   = 2000;
const int RPM_STEP  = 500;
const int MAP_ZONES = 6;
const int RPM_ZONES = 11;
int timingMap[MAP_ZONES][RPM_ZONES] = {
  {22, 24, 26, 30, 34, 38, 42, 42, 41, 40, 38},
  {20, 22, 24, 28, 32, 36, 38, 40, 39, 38, 36},
  {18, 20, 22, 26, 30, 33, 35, 36, 36, 35, 34},
  {16, 18, 20, 23, 26, 28, 30, 31, 31, 30, 29},
  {14, 16, 18, 21, 24, 26, 27, 28, 28, 27, 26},
  {12, 14, 16, 18, 20, 22, 23, 24, 25, 25, 26}
};

// ======= RPM & Smoothing =======
volatile unsigned long lastPulseMicros     = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile bool sawNewPulse                  = false;
const uint8_t NUM_LOBES                    = 4;
const float   DECEL_ALPHA                  = 0.3f;
const float   MAX_EXPECTED_RPM             = 9000.0f;
float rpmSmoothed = 0.0f;

// ======= Spark Scheduling =======
volatile bool  dwellPending       = false;
volatile bool  sparkPending       = false;
unsigned long  dwellStartMicros   = 0;
unsigned long  sparkEventMicros   = 0;
const unsigned long DWELL_US      = 3000;  // 3ms dwell

// ======= ISR: Distributor Pulse =======
void IRAM_ATTR pulseInterrupt() {
  unsigned long now = micros();
  if (lastPulseMicros) {
    pulseIntervalMicros = now - lastPulseMicros;
    sawNewPulse = true;
  }
  lastPulseMicros = now;
}

// ======= MAP Sensor =======
float readMAP() {
  const int samples = 10;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(MAP_SENSOR_PIN);
    delayMicroseconds(50);
  }
  float v = (sum / (float)samples) * (3.3f / 4095.0f);
  v = v * 2.0f * 1.5185f;
  float kPa = (v / 5.05f - 0.04f) / 0.0012858f;
  return constrain(kPa, 20.0f, 120.0f);
}

// ======= Lookup Advance =======
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
  Serial.begin(9600);
  pinMode(DISTRIBUTOR_PIN, INPUT);
  pinMode(MAP_SENSOR_PIN, INPUT);
  pinMode(IGNITION_COIL_PIN, OUTPUT);
  pinMode(HANDBRAKE_PIN, INPUT_PULLUP);
  digitalWrite(IGNITION_COIL_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(DISTRIBUTOR_PIN), pulseInterrupt, RISING);

  WiFi.softAP(apSsid, apPassword);
  server.begin();
  digitalWrite(LED_PIN, HIGH);
}

// ======= Main Loop =======
void loop() {
  // Handle WiFi client
  if (!client || !client.connected()) {
    WiFiClient nc = server.available();
    if (nc) client = nc;
  }

  // On distributor pulse, schedule dwell and spark
  if (sawNewPulse) {
    noInterrupts();
      unsigned long interval = pulseIntervalMicros;
      unsigned long pulseTime = lastPulseMicros;
      sawNewPulse = false;
    interrupts();

    if (interval > 5000 && interval < 2000000UL) {
      // RPM calculation
      float rawRpm = 120000000.0f / (interval * NUM_LOBES);
      if (rawRpm > MAX_EXPECTED_RPM) rawRpm = rpmSmoothed;
      rpmSmoothed = (rawRpm >= rpmSmoothed)
                  ? rawRpm
                  : DECEL_ALPHA * rawRpm + (1.0f - DECEL_ALPHA) * rpmSmoothed;
      int rpm = (int)rpmSmoothed;

      // Determine advance
      float mapVal = readMAP();
      int adv = getAdvance(rpm, mapVal);

      // Compute spark event time (BTDC) and dwell start
      unsigned long delayUs = (unsigned long)((interval * adv) / 720.0f);
      sparkEventMicros = pulseTime + delayUs;
      dwellStartMicros = (sparkEventMicros > DWELL_US)
                       ? (sparkEventMicros - DWELL_US)
                       : pulseTime;
      dwellPending = true;
      sparkPending = true;

      Serial.printf("Pulse@%lu int=%lu RPM=%d Adv=%d startDwell@%lu fire@%lu\n",
                    pulseTime, interval, rpm, adv, dwellStartMicros, sparkEventMicros);
    }
  }

  unsigned long now = micros();
  // Handle dwell start
  if (dwellPending && now >= dwellStartMicros) {
    dwellPending = false;
    digitalWrite(IGNITION_COIL_PIN, HIGH);
  }
  // Handle spark event
  if (sparkPending && now >= sparkEventMicros) {
    sparkPending = false;
    digitalWrite(IGNITION_COIL_PIN, LOW);
  }

  // Periodic status log
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 100) {
    int rpm = (int)rpmSmoothed;
    float mapVal = readMAP();
    int hb = digitalRead(HANDBRAKE_PIN) == LOW;
    char buf[128];
    snprintf(buf, sizeof(buf), "RPM:%d,MAP:%.1f,HB:%d T=%lu\n",
             rpm, mapVal, hb, millis());
    Serial.print(buf);
    if (client && client.connected()) client.print(buf);
    lastLog = millis();
  }
}
