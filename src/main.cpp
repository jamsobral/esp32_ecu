#include <Arduino.h>

const int ignitionPin = 5; // Confirm this matches your hardware

void setup() {
  Serial.begin(9600);
  pinMode(ignitionPin, OUTPUT);
  Serial.println("=== IGNITION SPARK TEST ===");
}

void loop() {
  digitalWrite(ignitionPin, HIGH);     // Trigger ignition (start dwell)
  Serial.println("SPARK!");            // Print for confirmation
  delay(5);                            // Hold HIGH for 5 ms
  digitalWrite(ignitionPin, LOW);      // End dwell (coil fires)
  delay(495);                          // Wait before next pulse (2Hz)
}
