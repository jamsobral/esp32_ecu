#include <Arduino.h>
#include <WiFi.h>

// Pin Definitions
const int hallPin = 4;
const int mapPin = 34;
const int ignitionPin = 5;
const int handbrakePin = 13;

// Trigger wheel setup
const int TEETH_COUNT = 36;
const int DEGREES_PER_TOOTH = 10;
const int TEETH_WITH_GAP = 35; // 36-1 wheel (missing tooth)
const int MISSING_TOOTH_TO_TDC_DEG = 190; // 19 teeth × 10° each

// Timing map [MAP (kPa)][RPM] -> Ignition advance (degrees BTDC)
int timingMap[5][12] = {
  {20, 22, 24, 26, 28, 30, 32, 34, 35, 35, 35, 35},
  {18, 20, 22, 24, 26, 28, 30, 32, 33, 34, 34, 34},
  {15, 17, 19, 21, 23, 25, 27, 29, 31, 32, 32, 32},
  {12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30, 30},
  {10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 29, 30}
};

// Wi-Fi
const char* WIFI_SSID = "ESP32_RPM";
const char* WIFI_PASSWORD = "esp32pass";
WiFiServer server(80);
WiFiClient client;

// State variables
volatile int toothCounter = -1;                // Tooth since missing tooth (0 = missing tooth)
volatile unsigned long lastToothTime = 0;      // microseconds
volatile unsigned long lastToothInterval = 0;  // microseconds
volatile bool gapDetected = false;
volatile bool scheduledSpark = false;
volatile unsigned long scheduledSparkTime = 0; // microseconds
volatile int lastFiredTooth = -1;              // To prevent double firing

// Tooth interval averaging
#define RPM_AVG_TEETH 2
volatile unsigned long toothIntervals[RPM_AVG_TEETH] = {0};
volatile int toothIntervalIdx = 0;

// Calculation results
float rpm = 0;
float mapValue = 100;
int targetAdvance = 20;
unsigned long periodPerTooth = 0; // microseconds

// Utility: MAP sensor reading
float readMAP() {
  const int numReadings = 5;
  long total = 0;
  for (int i = 0; i < numReadings; i++) {
    total += analogRead(mapPin);
    delayMicroseconds(100);
  }
  int raw = total / numReadings;
  float voltage = (raw / 4095.0) * 3.3;
  float kPa = (voltage - 0.2) * (700 - 15) / (4.7 - 0.2) + 15;
  return (raw < 300) ? 100.0 : kPa;
}

// Utility: Advance calculation
int getAdvance(int rpmVal, float mapVal) {
  int rpmIdx = constrain((rpmVal - 500) / 500, 0, 11);
  int mapIdx = constrain((int)(mapVal - 20) / 20, 0, 4);
  return constrain(timingMap[mapIdx][rpmIdx], 5, 35);
}

// Utility: RPM calculation
float calcRPM() {
  noInterrupts();
  unsigned long sum = 0;
  for (int i = 0; i < RPM_AVG_TEETH; i++) sum += toothIntervals[i];
  interrupts();
  float avgInterval = (float)sum / RPM_AVG_TEETH;
  if (avgInterval == 0) return 0;
  return 60000000.0 / (avgInterval * TEETH_WITH_GAP);
}

// Interrupt handler for each tooth
void IRAM_ATTR onHallPulse() {
  unsigned long now = micros();
  unsigned long interval = now - lastToothTime;
  toothIntervals[toothIntervalIdx] = interval;
  toothIntervalIdx = (toothIntervalIdx + 1) % RPM_AVG_TEETH;

  // Detect missing tooth (big gap)
  if (interval > lastToothInterval * 1.5) { // If needed, adjust sensitivity
    toothCounter = 0;
    gapDetected = true;
  } else if (toothCounter >= 0) {
    toothCounter++;
    if (toothCounter >= TEETH_WITH_GAP) toothCounter = 0; // Wrap at 35
  }

  lastToothInterval = interval;
  lastToothTime = now;
}

void setup() {
  Serial.begin(9600);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(ignitionPin, OUTPUT);
  pinMode(handbrakePin, INPUT_PULLUP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  server.begin();
  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, RISING);
}

// Helper: Schedule spark at exact microsecond delay after tooth event
void scheduleSpark(unsigned long delayUs) {
  scheduledSparkTime = micros() + delayUs;
  scheduledSpark = true;
}

void loop() {
  // Always handle new client connections
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) client = newClient;
  }

  static unsigned long lastCalc = 0;
  unsigned long now = millis();

  // Update RPM, MAP, advance every 100ms
  if (now - lastCalc >= 100) {
    rpm = calcRPM();
    mapValue = readMAP();
    targetAdvance = getAdvance(rpm, mapValue);

    // Calculate period per tooth in microseconds for delay calculation
    noInterrupts();
    unsigned long sum = 0;
    for (int i = 0; i < RPM_AVG_TEETH; i++) sum += toothIntervals[i];
    unsigned long avgInterval = sum / RPM_AVG_TEETH;
    interrupts();
    periodPerTooth = avgInterval;

    lastCalc = now;
  }

  bool cutIgnition = digitalRead(handbrakePin) == LOW;

  // Spark scheduling logic (triggered on new missing tooth detection)
  static int firingTooth = -1;
  static int lastAdvance = 0;

  if (gapDetected && toothCounter >= 0 && rpm > 100) {
    // Calculate which tooth after missing tooth to fire
    int tdcTooth = MISSING_TOOTH_TO_TDC_DEG / DEGREES_PER_TOOTH; // usually 19
    int fullTeethAdvance = targetAdvance / DEGREES_PER_TOOTH;     // integer teeth
    int remAdvance = targetAdvance % DEGREES_PER_TOOTH;           // remaining degrees
    firingTooth = (tdcTooth - fullTeethAdvance + TEETH_WITH_GAP) % TEETH_WITH_GAP;
    lastAdvance = remAdvance;
    gapDetected = false;
    lastFiredTooth = -1; // reset to allow firing for this cycle
  }

  // Check if we're on the firing tooth (real-time, every loop)
  noInterrupts();
  int currentTooth = toothCounter;
  interrupts();

  // If on the firing tooth, schedule the spark for the correct moment (sub-tooth)
  if (!cutIgnition && currentTooth == firingTooth && currentTooth != lastFiredTooth && rpm > 100 && periodPerTooth > 0) {
    // Compute delay for the fractional advance (sub-tooth)
    // Time per degree = periodPerTooth / 10
    unsigned long microDelay = lastAdvance * (periodPerTooth / DEGREES_PER_TOOTH);

    scheduleSpark(microDelay);
    lastFiredTooth = currentTooth;
    // Debug: Serial.println("Spark scheduled");
  }

  // Fire the spark if scheduled and time has arrived
  if (scheduledSpark && (long)(micros() - scheduledSparkTime) >= 0) {
    digitalWrite(ignitionPin, HIGH);
    delayMicroseconds(1000); // 1ms spark pulse, adjust as needed
    digitalWrite(ignitionPin, LOW);
    scheduledSpark = false;
    Serial.println("SPARK!");
  }

  // Debug output
  Serial.printf("RPM: %.1f, MAP: %.1f, Adv: %d, Tooth: %d, FireTooth: %d\n", rpm, mapValue, targetAdvance, currentTooth, firingTooth);

  if (client.connected()) {
    client.printf("RPM: %.1f MAP: %.1f Adv: %d Tooth: %d FireTooth: %d\n", rpm, mapValue, targetAdvance, currentTooth, firingTooth);
  }

  delay(2);
}
