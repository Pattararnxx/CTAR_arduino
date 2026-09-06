#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_system.h>

NAU7802 myScale;

// ===== UUID =====
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ===== I2C =====
const int I2C_SDA = 20;
const int I2C_SCL = 18;

// ===== Battery =====
const int BATTERY_PIN = A1;

// แรงดันขั้นต่ำที่ถือว่า "มีแบตเตอรี่"
// ใช้แค่ตรวจว่ามีแบตหรือไม่ ไม่ได้ใช้ตรวจ Charging
const float BATTERY_PRESENT_VOLTAGE = 2.70f;

// ===== Status LED =====
#define STATUS_LED 15   // built-in yellow LED, active-low
                         // LOW  = ON
                         // HIGH = OFF

unsigned long lastBlink = 0;
bool ledState = false;

// ===== BLE =====
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// ===== Calibration =====

// float CALIBRATION_FACTOR = 326500;

float CALIBRATION_FACTOR = 326496;


// ===== Filter =====
const float ALPHA = 0.2;
float filtered_weight = 0.0;
float filtered_force = 0.0;

// ===== Battery Filter =====
float filteredVoltage = 0.0;
const float VOLTAGE_ALPHA = 0.2;


// =====================================================
// อ่านแรงดันแบตเตอรี่
// =====================================================
float readBatteryVoltage() {

  uint32_t sum = 0;

  for (int i = 0; i < 64; i++) {
    sum += analogReadMilliVolts(BATTERY_PIN);
    delay(2);
  }

  float adcVoltage = sum / 64.0f;

  // Voltage divider 1:2
  return (adcVoltage * 2.0f) / 1000.0f;
}


// =====================================================
// คำนวณ Battery Percentage
// =====================================================
uint8_t batteryPercent(float v) {

  if (v >= 3.30f)
    return 100;

  if (v <= 2.70f)
    return 0;

  const float voltageTable[] = {
    3.30,
    3.2333,
    3.1667,
    3.10,
    3.0333,
    2.9667,
    2.90,
    2.8333,
    2.70
  };

  const int percentTable[] = {
    100,
    90,
    80,
    65,
    45,
    25,
    10,
    5,
    0
  };

  const int N =
      sizeof(voltageTable) /
      sizeof(float);

  for (int i = 0; i < N - 1; i++) {

    if (v <= voltageTable[i] &&
        v > voltageTable[i + 1]) {

      float v1 = voltageTable[i];
      float v2 = voltageTable[i + 1];

      int p1 = percentTable[i];
      int p2 = percentTable[i + 1];

      float percent =
          p1 +
          (v - v1) *
          (p2 - p1) /
          (v2 - v1);

      return (uint8_t)(percent + 0.5f);
    }
  }

  return 0;
}


// =====================================================
// ตรวจว่ามี Battery หรือไม่
//
// ใช้แรงดันแค่ตรวจ "มีแบต"
// ไม่ได้ใช้แรงดันตรวจว่า USB/Charging
// =====================================================
bool isBatteryPresent(float voltage) {

  return voltage >= BATTERY_PRESENT_VOLTAGE;
}


// =====================================================
// ตรวจ USB ผ่าน Serial
// =====================================================
bool isUSBConnected() {

  return Serial;
}


// =====================================================
// Status LED
//
// USB + Battery + <98%  = กระพริบ
// USB + Battery + >=98% = ติดค้าง
// ไม่มี USB              = ดับ
// ไม่มี Battery          = ดับ
// =====================================================
void updateStatusLED(uint8_t percent, float batteryVoltage) {

  bool usbConnected = isUSBConnected();
  bool batteryPresent = isBatteryPresent(batteryVoltage);

  // // ===== Debug =====
  // Serial.print("[STATUS] USB: ");
  // Serial.print(usbConnected ? "CONNECTED" : "NOT CONNECTED");

  // Serial.print(" | Battery: ");
  // Serial.print(batteryPresent ? "PRESENT" : "NOT PRESENT");

  // Serial.print(" | Voltage: ");
  // Serial.print(batteryVoltage, 2);

  // Serial.print("V | ");
  // Serial.print(percent);

  // Serial.println("%");


  // =================================================
  // ไม่มีแบต
  // =================================================
  if (!batteryPresent) {

    digitalWrite(STATUS_LED, HIGH);

    ledState = false;

    return;
  }


  // =================================================
  // ไม่มี USB
  // =================================================
  if (!usbConnected) {

    digitalWrite(STATUS_LED, HIGH);

    ledState = false;

    return;
  }


  // =================================================
  // มี USB + มีแบต + Battery Full
  // =================================================
  if (percent >= 98) {

    digitalWrite(STATUS_LED, LOW);

    ledState = true;

    return;
  }


  // =================================================
  // มี USB + มีแบต + Battery ยังไม่เต็ม
  // → Charging
  // =================================================

  if (millis() - lastBlink >= 500) {

    ledState = !ledState;

    if (ledState) {
      digitalWrite(STATUS_LED, LOW);
    }
    else {
      digitalWrite(STATUS_LED, HIGH);
    }

    lastBlink = millis();
  }
}


// =====================================================
// Packet
// =====================================================
struct DataPacket {

  float force;
  uint8_t battery;
};


// =====================================================
// BLE Callback
// =====================================================
class MyServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer* pServer) {

    deviceConnected = true;

    Serial.println("Device Connected!");
  }


  void onDisconnect(BLEServer* pServer) {

    deviceConnected = false;

    Serial.println("Device Disconnected!");

    BLEDevice::startAdvertising();
  }
};


// =====================================================
// Setup BLE
// =====================================================
void setupBLE() {

  BLEDevice::init("CTAR_Prototype");

  pServer = BLEDevice::createServer();

  pServer->setCallbacks(
    new MyServerCallbacks()
  );

  BLEService *pService =
      pServer->createService(
        SERVICE_UUID
      );

  pCharacteristic =
      pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
      );

  pCharacteristic->addDescriptor(
    new BLE2902()
  );

  pService->start();

  BLEAdvertising *pAdvertising =
      BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(
    SERVICE_UUID
  );

  pAdvertising->setScanResponse(false);

  pAdvertising->setMinPreferred(0x0);

  BLEDevice::startAdvertising();

  Serial.println(
    "BLE Ready! Waiting for connection..."
  );
}


// =====================================================
// Setup
// =====================================================
void setup() {

  Serial.begin(115200);

  // ===== Battery =====
  pinMode(BATTERY_PIN, INPUT);

  analogReadResolution(12);

  analogSetPinAttenuation(
    BATTERY_PIN,
    ADC_11db
  );


  // ===== Status LED =====
  pinMode(
    STATUS_LED,
    OUTPUT
  );

  // LED OFF
  digitalWrite(
    STATUS_LED,
    HIGH
  );


  Serial.println();
  Serial.println(
    "เริ่มระบบวัดแรง CTAR (NAU7802)..."
  );


  // ===== I2C =====
  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );


  // ===== NAU7802 =====
  if (!myScale.begin()) {

    Serial.println(
      "ไม่พบ NAU7802"
    );

    while (1);
  }

  Serial.println(
    "พบ NAU7802 แล้ว"
  );


  myScale.setGain(
    NAU7802_GAIN_128
  );

  myScale.setSampleRate(
    NAU7802_SPS_80
  );

  myScale.calibrateAFE();


  // ===== Tare =====
  Serial.println(
    "กำลัง Tare..."
  );

  delay(1000);

  myScale.calculateZeroOffset();

  Serial.println(
    "Tare เสร็จ"
  );


  // ===== Calibration =====
  myScale.setCalibrationFactor(
    CALIBRATION_FACTOR
  );


  // ===== BLE =====
  setupBLE();
}


// =====================================================
// Loop
// =====================================================
void loop() {

  if (myScale.available()) {

    // =================================================
    // อ่าน Weight
    // =================================================

    float raw_weight =
        myScale.getWeight();

    float raw =
        myScale.getReading();


    if (raw_weight < 0.0)
      raw_weight = 0.0;


    // =================================================
    // EMA Filter
    // =================================================

    filtered_weight =
        (ALPHA * raw_weight) +
        ((1.0 - ALPHA) * filtered_weight);


    filtered_force =
        filtered_weight * 9.8;


    // =================================================
    // Reset เมื่อแรงเกิน
    // =================================================

    if (filtered_weight > 10.0) {

      Serial.println(
        "Force > 10kg → Restart"
      );

      delay(500);

      esp_restart();
    }


    // =================================================
    // Debug Weight
    // =================================================

    Serial.print("Raw:");
    Serial.print(raw);

    Serial.print(" W:");
    Serial.print(
      filtered_weight,
      3
    );

    Serial.print(" F:");
    Serial.println(
      filtered_force,
      3
    );


    // =================================================
    // อ่าน Battery
    // =================================================

    float rawBatteryVoltage =
        readBatteryVoltage();


    // ค่าเริ่มต้น
    if (filteredVoltage == 0.0) {

      filteredVoltage =
          rawBatteryVoltage;
    }


    // EMA Filter
    filteredVoltage =
        (VOLTAGE_ALPHA *
         rawBatteryVoltage)
        +
        ((1.0 - VOLTAGE_ALPHA) *
         filteredVoltage);


    // =================================================
    // Battery %
    // =================================================

    uint8_t battery =
        batteryPercent(
          filteredVoltage
        );


    // =================================================
    // Update LED
    // =================================================

    updateStatusLED(
      battery,
      filteredVoltage
    );


    // Serial.print("Battery: ");

    // Serial.print(
    //   filteredVoltage,
    //   2
    // );

    // Serial.print("V (");

    // Serial.print(
    //   battery
    // );

    // Serial.println("%)");


    // =================================================
    // BLE
    // =================================================

    if (deviceConnected) {

      float safe_force =
          (filtered_force < 0.0f)
          ? 0.0f
          : filtered_force;


      DataPacket data = {
        safe_force,
        battery
      };


      pCharacteristic->setValue(
        (uint8_t*)&data,
        sizeof(data)
      );


      pCharacteristic->notify();


      Serial.print(
        "BLE SEND: force="
      );

      Serial.print(
        safe_force,
        3
      );

      Serial.print(
        " battery="
      );

      Serial.println(
        battery
      );
    }


    delay(10);
  }
}
