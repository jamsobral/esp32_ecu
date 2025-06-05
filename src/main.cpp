#include <Arduino.h>
#include <WiFi.h>

const int HALL_PIN                    = 4;      // Hall sensor input
const uint8_t PULSES_PER_REV          = 35;     // 36-1 wheel -> 35 pulses/rev
const unsigned long UPDATE_INTERVAL_MS = 200;   // Send RPM every 200 ms

const char* WIFI_SSID     = "ESP32_RPM";          // WiFi Access Point SSID
const char* WIFI_PASSWORD = "esp32pass";          // Soft AP password

// Pulse counter shared between ISR and loop
volatile unsigned long pulseCount = 0;

WiFiServer server(80);
WiFiClient client;

void IRAM_ATTR onHallPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onHallPulse, RISING);

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  server.begin();
}

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

  float rpm = (pulses * (60000.0f / UPDATE_INTERVAL_MS)) / PULSES_PER_REV;

  Serial.printf("RPM: %.2f\n", rpm);
  if (client && client.connected()) {
    client.printf("%.2f\n", rpm);
  }
}
