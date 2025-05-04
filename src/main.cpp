#include <Arduino.h>

#define IGNITION_COIL_PIN  5  // Adjust this if you're using another pin

void setup() {
  pinMode(IGNITION_COIL_PIN, OUTPUT);
  digitalWrite(IGNITION_COIL_PIN, LOW);
}

void loop() {
  // Fire the coil (2ms dwell)
  digitalWrite(IGNITION_COIL_PIN, HIGH);
  delayMicroseconds(2000);
  digitalWrite(IGNITION_COIL_PIN, LOW);

  // Wait before next pulse
  delay(500);  // 500ms between sparks (2 sparks/sec)
}