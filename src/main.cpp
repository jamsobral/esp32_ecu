#include <Arduino.h>
#define IGNITION_COIL_PIN 5      // GPIO controlling the coil's ground via MOSFET
#define DWELL_TIME_US     3000   // Dwell time: 3 ms (charging the coil)
#define FIRE_INTERVAL_MS  2000   // Fire every 2 seconds

void setup() {
  Serial.begin(9600);
  pinMode(IGNITION_COIL_PIN, OUTPUT);
  digitalWrite(IGNITION_COIL_PIN, LOW);
  Serial.println("Spark test started. Coil will fire every 2 seconds.");
}

void loop() {
  Serial.println("Charging coil...");
  digitalWrite(IGNITION_COIL_PIN, HIGH);
  delayMicroseconds(DWELL_TIME_US);

  Serial.println("Firing coil (spark)!");
  digitalWrite(IGNITION_COIL_PIN, LOW);

  delay(FIRE_INTERVAL_MS);
}
