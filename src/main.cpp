#include <Arduino.h>
#include <WiFi.h>

// ======= Hardware Pins =======
#define DISTRIBUTOR_PIN     4    // Input from VR conditioner
#define HANDBRAKE_PIN       13   // Handbrake switch input
#define IGNITION_COIL_PIN   5    // Output to MOSFET gate
#define MAP_SENSOR_PIN      34   // Analog input from MAP sensor
#define LED_PIN             2    // On-board LED for status indication

// ======= WiFi & TCP Server =======
const char* ssid     = "ESP32_Ignition";
const char* password = "e3012345678";
WiFiServer tcpServer(80);
WiFiClient client;

// ======= Timing & Ignition Map =======
volatile unsigned long lastPulseTime = 0;
volatile unsigned long engineRpm     = 0;

const int RPM_STEPS   = 14;    // 500 RPM increments up to 7000 RPM
const int MAP_STEPS   = 5;
const unsigned long rpmBreakpoints[RPM_STEPS] = {500,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000};
const float mapBreakpoints[MAP_STEPS] = {10.0,30.0,50.0,70.0,100.0};
int ignitionMap[RPM_STEPS][MAP_STEPS] = {
  {5,7,9,11,13},{6,8,10,12,14},{8,10,12,14,16},{10,12,14,16,18},
  {12,14,16,18,20},{14,16,18,20,22},{16,18,20,22,24},{18,20,22,24,26},
  {19,21,23,25,27},{20,22,24,26,28},{21,23,25,27,29},{22,24,26,28,30},
  {23,25,27,29,31},{24,26,28,30,32}
};

// ======= Interrupt Handler =======
void IRAM_ATTR handlePulse() {
  unsigned long now = micros();
  unsigned long delta = now - lastPulseTime;
  if (delta > 200) {
    engineRpm = 60000000UL / delta;
    lastPulseTime = now;
  }
}

// ======= Helper: Linear Interpolation =======
float interp(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) return y0;
  return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

int lookupAdvance(float kPa, unsigned long rpm) {
  int i;
  for (i = 1; i < RPM_STEPS; i++) if (rpm < rpmBreakpoints[i]) break;
  int i0 = max(0, i-1), i1 = min(i, RPM_STEPS-1);
  int j;
  for (j = 1; j < MAP_STEPS; j++) if (kPa < mapBreakpoints[j]) break;
  int j0 = max(0, j-1), j1 = min(j, MAP_STEPS-1);
  float a00 = ignitionMap[i0][j0], a10 = ignitionMap[i1][j0];
  float a01 = ignitionMap[i0][j1], a11 = ignitionMap[i1][j1];
  float rpm0 = rpmBreakpoints[i0], rpm1 = rpmBreakpoints[i1];
  float map0 = mapBreakpoints[j0], map1 = mapBreakpoints[j1];
  float adv0 = interp(rpm, rpm0, rpm1, a00, a10);
  float adv1 = interp(rpm, rpm0, rpm1, a01, a11);
  return round(interp(kPa, map0, map1, adv0, adv1));
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  // Boot indication: blink LED 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  Serial.begin(115200);
  pinMode(DISTRIBUTOR_PIN, INPUT);
  pinMode(HANDBRAKE_PIN, INPUT_PULLDOWN);
  pinMode(IGNITION_COIL_PIN, OUTPUT);
  pinMode(MAP_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DISTRIBUTOR_PIN), handlePulse, RISING);
  digitalWrite(IGNITION_COIL_PIN, LOW);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  tcpServer.begin();
  // Turn LED solid on when WiFi connected
  digitalWrite(LED_PIN, HIGH);
  Serial.printf("WiFi connected, IP: %s", WiFi.localIP().toString().c_str());
}

void loop() {
  // Manage client connection LED
  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    client = newClient;
    // Blink LED to indicate new connection
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
  }

  // Read sensors
  float mapRaw = analogRead(MAP_SENSOR_PIN) / 4095.0 * 500.0;
  unsigned long rpm = engineRpm;
  int advanceDegrees = lookupAdvance(mapRaw, rpm);

  // Fire ignition
  digitalWrite(IGNITION_COIL_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(IGNITION_COIL_PIN, LOW);

  // Logging
  String logMsg = String("RPM:") + rpm + ", MAP:" + mapRaw + ", ADV:" + advanceDegrees;
  Serial.println(logMsg);
  if (client && client.connected()) {
    client.println(logMsg);
  }

  // Delay until next event
  unsigned long intervalUs = 60000000UL / max(rpm,1UL);
  delayMicroseconds(intervalUs - 1000);
}