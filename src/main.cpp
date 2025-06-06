#include <Arduino.h>

const int ignitionPin = 5;

void setup() {
  Serial.begin(9600);
  pinMode(ignitionPin, OUTPUT);
  Serial.println("=== IGNITION SPARK TEST ===");
}

void loop() {
  digitalWrite(ignitionPin, LOW);   // MOSFET ON (start dwell, charge coil)
  delay(5);                          // 5 ms dwell (safe)
  digitalWrite(ignitionPin, HIGH);    // MOSFET OFF (coil fires)
  Serial.println("SPARK!");
  delay(500);                        // Wait before next pulse (500 ms)
}
