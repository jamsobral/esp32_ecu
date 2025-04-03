#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>

// Pin Definitions
const int hallPin = 4;        // Hall sensor signal (stock loom, with 10kΩ + 3.3kΩ divider)
const int mapPin = 34;        // MPX5700AP MAP sensor (ADC)
const int ignitionPin = 5;    // Main ignition coil (via IRLZ44N MOSFET)
const int handbrakePin = 6;   // Handbrake switch (normally open, closes to GND when up)

// Variables for RPM calculation
volatile unsigned long lastPulse = 0;
volatile unsigned long period = 0;
volatile int rpm = 0;

// Timing map: [MAP (kPa)][RPM] -> Ignition advance (degrees BTDC)
// Rows: MAP (20, 50, 70, 90, 100 kPa)
// Columns: RPM (500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000 RPM)
int timingMap[5][12] = {
  {12, 12, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18},
  {12, 12, 14, 15, 16, 17, 18, 20, 22, 24, 26, 26},
  {12, 12, 14, 15, 16, 18, 20, 22, 24, 26, 27, 27},
  {10, 10, 12, 13, 14, 15, 16, 18, 20, 22, 24, 24},
  {10, 10, 11, 12, 13, 14, 16, 18, 20, 22, 24, 25}
};

// EEPROM settings
#define EEPROM_SIZE 240

// BLE UUIDs
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;
unsigned long lastAdvertiseCheck = 0;
String incomingCommand = "";

// Debug blink function
void blinkLED(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(2, HIGH);
    delay(duration);
    digitalWrite(2, LOW);
    delay(duration);
  }
}

// Save timing map to EEPROM
void saveTimingMap() {
  int address = 0;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 12; j++) {
      EEPROM.writeInt(address, timingMap[i][j]);
      address += sizeof(int);
    }
  }
  EEPROM.commit();
  if (deviceConnected) {
    String message = "Timing map saved to EEPROM\n";
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
  }
}

// Load timing map from EEPROM
void loadTimingMap() {
  int address = 0;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 12; j++) {
      timingMap[i][j] = EEPROM.readInt(address);
      address += sizeof(int);
      if (timingMap[i][j] < 5 || timingMap[i][j] > 35) {
        timingMap[i][j] = timingMap[i][j];
      }
    }
  }
}

// Interrupt handler for Hall sensor (RPM calculation)
void IRAM_ATTR pulseInterrupt() {
  unsigned long now = micros();
  period = now - lastPulse;
  lastPulse = now;
}

// Read MAP sensor (MPX5700AP, 0-700 kPa, 0.2-4.7V output) with averaging
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
  return constrain(kPa, 20, 100);
}

// Calculate ignition advance based on RPM and MAP
int getAdvance(int rpm, float map) {
  int rpmIdx = constrain((rpm - 500) / 500, 0, 11);
  int mapIdx = constrain((int)(map - 20) / 20, 0, 4);
  int baseAdvance = timingMap[mapIdx][rpmIdx];
  return constrain(baseAdvance, 5, 35);
}

// Process BLE commands to adjust timing map
void processCommand(String command) {
  if (command.startsWith("SET_MAP")) {
    int mapIdx, rpmIdx, value;
    sscanf(command.c_str(), "SET_MAP %d %d %d", &mapIdx, &rpmIdx, &value);
    if (mapIdx >= 0 && mapIdx < 5 && rpmIdx >= 0 && rpmIdx < 12 && value >= 5 && value <= 35) {
      timingMap[mapIdx][rpmIdx] = value;
      String message = "Updated timingMap[" + String(mapIdx) + "][" + String(rpmIdx) + "] = " + String(value) + "\n";
      pTxCharacteristic->setValue(message.c_str());
      pTxCharacteristic->notify();
      saveTimingMap();
    } else {
      String message = "Invalid SET_MAP command. Use: SET_MAP mapIdx rpmIdx value (e.g., SET_MAP 0 0 25)\n";
      pTxCharacteristic->setValue(message.c_str());
      pTxCharacteristic->notify();
    }
  } else if (command == "SAVE_MAP") {
    saveTimingMap();
  } else if (command == "RESET_MAP") {
    int defaultMap[5][12] = {
      {12, 12, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18},
      {12, 12, 14, 15, 16, 17, 18, 20, 22, 24, 26, 26},
      {12, 12, 14, 15, 16, 18, 20, 22, 24, 26, 27, 27},
      {10, 10, 12, 13, 14, 15, 16, 18, 20, 22, 24, 24},
      {10, 10, 11, 12, 13, 14, 16, 18, 20, 22, 24, 25}
    };
    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 12; j++) {
        timingMap[i][j] = defaultMap[i][j];
      }
    }
    saveTimingMap();
    String message = "Timing map reset to default values\n";
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
  } else {
    String message = "Unknown command. Use: SET_MAP mapIdx rpmIdx value, SAVE_MAP, or RESET_MAP\n";
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
  }
}

// Callback for connection/disconnection events
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    blinkLED(3, 50);
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    blinkLED(5, 500);
    BLEDevice::startAdvertising();
  }
};

// Callback for receiving data via BLE
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++) {
        char c = rxValue[i];
        if (c == '\n') {
          processCommand(incomingCommand);
          incomingCommand = "";
        } else {
          incomingCommand += c;
        }
      }
      blinkLED(3, 50);
    }
  }
};

void setup() {
  // Step 1: Blink 1 time - Start of setup
  pinMode(2, OUTPUT);
  blinkLED(1, 200);

  // Step 2: Blink 2 times - After EEPROM init
  EEPROM.begin(EEPROM_SIZE);
  loadTimingMap();
  blinkLED(2, 200);

  // Step 3: Blink 3 times - After pin setup
  pinMode(hallPin, INPUT);
  pinMode(ignitionPin, OUTPUT);
  pinMode(handbrakePin, INPUT_PULLUP);
  blinkLED(3, 200);

  // Step 4: Blink 4 times - After interrupt attach
  attachInterrupt(digitalPinToInterrupt(hallPin), pulseInterrupt, FALLING);
  blinkLED(4, 200);

  // Initialize BLE
  BLEDevice::init("ESP32_Ignition");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create TX Characteristic (for sending data to phone)
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create RX Characteristic (for receiving commands from phone)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  // Step 5: Blink 5 times - BLE initialized successfully
  blinkLED(5, 200);
}

unsigned long nextSpark = 0;

void loop() {
  // Update RPM from interrupt data
  if (period > 0) {
    rpm = 60000000 / period;
  } else {
    rpm = 0;
  }

  // Read sensors
  float map = readMAP();
  int advance = getAdvance(rpm, map);
  bool cutIgnition = (digitalRead(handbrakePin) == LOW);

  // Fire ignition coil if handbrake isn't active
  if (!cutIgnition && micros() >= nextSpark && rpm > 0) {
    digitalWrite(ignitionPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(ignitionPin, LOW);
    unsigned long period = 60000000 / rpm;
    unsigned long advanceTime = (period * advance) / 360;
    nextSpark = micros() + period - advanceTime;
  }

  // Send data every 100ms if connected
  static unsigned long lastUpdate = 0;
  if (deviceConnected && millis() - lastUpdate >= 100) {
    String message = "RPM: " + String(rpm) + " MAP: " + String(map) + " Advance: " + String(advance) + " Handbrake: " + (cutIgnition ? "ON" : "OFF") + "\n";
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
    lastUpdate = millis();
  }

  // Check if advertising needs to be restarted (every 30 seconds)
  if (!deviceConnected && millis() - lastAdvertiseCheck >= 30000) {
    BLEDevice::startAdvertising();
    blinkLED(3, 50);
    lastAdvertiseCheck = millis();
  }

  // Blink LED to indicate state
  if (deviceConnected) {
    digitalWrite(2, HIGH);
    delay(50);
    digitalWrite(2, LOW);
    delay(50);
  } else {
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
  }

  delay(10); // Small delay to give BLE stack time to process events
}