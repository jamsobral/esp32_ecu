#include <Arduino.h>

const int mapPin = 34; // ADC1_6 on ESP32, confirm your wiring

void setup() {
  Serial.begin(9600);
  pinMode(mapPin, INPUT);
  Serial.println("=== MAP Sensor Test ===");
}

void loop() {
  const int numReadings = 10;
  long total = 0;
  for (int i = 0; i < numReadings; i++) {
    total += analogRead(mapPin);
    delay(5);
  }
  int raw = total / numReadings;
  float voltage = (raw / 4095.0) * 3.3;
  float kPa = (voltage - 0.2) * (700.0 - 15.0) / (4.7 - 0.2) + 15.0;

  Serial.printf("Raw ADC: %4d\tVoltage: %.3f V\tMAP: %.2f kPa\n", raw, voltage, kPa);
  delay(250);
}
