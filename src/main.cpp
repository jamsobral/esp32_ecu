#include <Arduino.h>

// Pin Definitions
const int hallPin = 4;

// Trigger wheel setup
const int TEETH_COUNT = 36;
const int TEETH_WITH_GAP = 35; // 36-1 wheel

// State variables
volatile int toothCounter = -1;
volatile unsigned long lastToothTime = 0;
volatile unsigned long lastToothInterval = 0;

void IRAM_ATTR onHallPulse() {
  unsigned long now = micros();
  unsigned long interval = now - lastToothTime;

  // Detect missing tooth
  if (interval > lastToothInterval * 1.3 && lastToothInterval > 1000) { // 1.3× threshold, ignore first pulse
    toothCounter = 0;
    Serial.print("=== MISSING TOOTH DETECTED! ===\n");
  } else if (toothCounter >= 0) {
    toothCounter++;
    if (toothCounter >= TEETH_WITH_GAP) toothCounter = 0;
  }

  lastToothInterval = interval;
  lastToothTime = now;

  // Print every tooth event
  Serial.printf("Tooth: %2d   Interval: %6lu us\n", toothCounter, interval);
}

void setup() {
  Serial.begin(9600);
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, RISING);
  Serial.println("=== Trigger Wheel Test ===");
}

void loop() {
  delay(100);
}
