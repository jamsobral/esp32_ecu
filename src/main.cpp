#include <Arduino.h>

const int CAM_PIN                = 4;      // GPIO from MAX9926 Out1
const uint8_t NUM_LOBES          = 4;      // Number of lobes on your distributor spider
const unsigned long UPDATE_INTERVAL_MS = 100;      // How often to refresh/print (ms)
const float DECEL_ALPHA          = 0.3f;   // 0 = max hang, 1 = no hang
const float MAX_EXPECTED_RPM     = 8000.0f; // ignore any rawRPM > this

// Shared between ISR and loop
volatile unsigned long lastPulseMicros    = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile bool sawNewPulse                 = false;

void IRAM_ATTR onCamPulse() {
  unsigned long now = micros();
  if (lastPulseMicros != 0) {
    pulseIntervalMicros = now - lastPulseMicros;
    sawNewPulse = true;
  }
  lastPulseMicros = now;
}

void setup() {
  Serial.begin(9600);
  pinMode(CAM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAM_PIN), onCamPulse, RISING);
}

void loop() {
  static unsigned long lastPrintMs = 0;
  static float        displayedRPM = 0.0f;

  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs < UPDATE_INTERVAL_MS) return;
  lastPrintMs = nowMs;

  // Grab the latest interval & flag, then clear sawNewPulse
  noInterrupts();
    unsigned long interval = pulseIntervalMicros;
    bool havePulse         = sawNewPulse;
    sawNewPulse           = false;
  interrupts();

  if (havePulse && interval > 0) {
    // 120e6 = 2 * 60 * 10^6 (cam→crank & µs→min)
    float rawRPM = 120000000.0f / (interval * NUM_LOBES);

    // --- clamp any crazy spikes above MAX_EXPECTED_RPM ---
    if (rawRPM > MAX_EXPECTED_RPM) {
      rawRPM = displayedRPM;  // ignore this spike
    }

    // --- rev‑hang smoothing on decel only ---
    if (rawRPM >= displayedRPM) {
      displayedRPM = rawRPM;  // jump up immediately
    } else {
      // blend toward new value
      displayedRPM = DECEL_ALPHA * rawRPM
                   + (1.0f - DECEL_ALPHA) * displayedRPM;
    }
  }

  Serial.printf("Time: %lu ms | RPM: %.2f\n", nowMs, displayedRPM);
}
