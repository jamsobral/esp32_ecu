#include <Arduino.h>   // (or) #include <stdint.h>

const int CAM_PIN = 4;          // match your wiring
volatile uint32_t pulses = 0;

void IRAM_ATTR onCam() {
  pulses++;
}

void setup() {
  Serial.begin(115200);
  pinMode(CAM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAM_PIN), onCam, RISING);
}

void loop() {
  static uint32_t lastTime = 0;
  if (millis() - lastTime >= 1000) {
    // If your distributor “star” has e.g. 6 lobes:
    const uint8_t N = 6;
    float rpm = (pulses / float(N)) * 2 * 60;
    Serial.printf("RPM ≈ %.0f  (raw: %u)\n", rpm, pulses);
    pulses = 0;
    lastTime = millis();
  }
}
