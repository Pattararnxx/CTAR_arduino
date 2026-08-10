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

// const int DRDY = 10;
const int BATTERY_PIN = A1;   // GPIO2 = A0.

// ===== Status LED =====
#define STATUS_LED 15   // built-in yellow LED, active-low (LOW = ติด)
float lastVoltage = 0.0;
unsigned long lastBlink = 0;
bool ledState = false;

// ===== BLE =====
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// ===== Interrupt =====
// volatile bool dataReady = false;

// ===== Calibration =====
float CALIBRATION_FACTOR = 715800;

// ===== Filter =====
const float ALPHA = 0.2;
float filtered_weight = 0.0;
float filtered_force = 0.0;

float readBatteryVoltage() {
  uint32_t sum = 0;

  for (int i = 0; i < 32; i++) {
    sum += analogReadMilliVolts(BATTERY_PIN);
    delay(2);
  }

  float adcVoltage = sum / 32.0f;

  return (adcVoltage * 2.0f) / 1000.0f;
}

uint8_t batteryPercent(float v) {

  if (v >= 4.20f) return 100;
  if (v <= 3.30f) return 0;

  const float voltageTable[] = {
    4.20, 4.10, 4.00, 3.90,
    3.80, 3.70, 3.60, 3.50,
    3.30
  };

  const int percentTable[] = {
    100, 90, 80, 65,
    45, 25, 10, 5,
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

// ===== Status LED: กะพริบ = กำลังชาร์จ, ติดค้าง = แบตเต็ม, ดับ = ปกติ =====
void updateStatusLED(float voltage, uint8_t percent) {
  bool full = (percent >= 98);
  bool charging = (voltage > lastVoltage + 0.01);

  if (full) {
    digitalWrite(STATUS_LED, LOW); // ติดค้าง
  } else if (charging) {
    if (millis() - lastBlink > 500) {
      ledState = !ledState;
      digitalWrite(STATUS_LED, ledState ? LOW : HIGH);
      lastBlink = millis();
    }
  } else {
    digitalWrite(STATUS_LED, HIGH); // ดับ
  }

  lastVoltage = voltage;
}

// ===== Packet (ส่งหลายค่าแบบ binary) =====
struct DataPacket {
  float force;
  uint8_t battery;
};

// ===== Interrupt Service Routine =====
// void IRAM_ATTR onDataReady() {
//   dataReady = true;
// }

// ===== BLE Callback =====
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device Connected!");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device Disconnected!");
    BLEDevice::startAdvertising();
  }
};

// ===== Setup BLE =====
void setupBLE() {
  BLEDevice::init("CTAR_Prototype");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);

  BLEDevice::startAdvertising();

  Serial.println("BLE Ready! Waiting for connection...");
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH); // ปิดไว้ก่อน
  lastVoltage = readBatteryVoltage(); // baseline สำหรับเช็คการชาร์จ

  Serial.println("เริ่มระบบวัดแรง CTAR (NAU7802)...");

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!myScale.begin()) {
    Serial.println("ไม่พบ NAU7802");
    while (1);
  }

  Serial.println("พบ NAU7802 แล้ว");

  myScale.setGain(NAU7802_GAIN_1);
  myScale.setSampleRate(NAU7802_SPS_80);
  myScale.calibrateAFE();

  Serial.println("กำลัง Tare...");
  delay(1000);
  myScale.calculateZeroOffset();
  Serial.println("Tare เสร็จ");

  myScale.setCalibrationFactor(CALIBRATION_FACTOR);
  setupBLE();
}

// ===== Loop =====
void loop() {

  if (myScale.available()) {

    float raw_weight = myScale.getWeight();
    float raw = myScale.getReading();

    if (raw_weight < 0.0) raw_weight = 0.0;

    // EMA Filter
    filtered_weight = (ALPHA * raw_weight) + ((1.0 - ALPHA) * filtered_weight);
    filtered_force = filtered_weight * 9.8;

    // ===== Reset เมื่อแรงเกิน =====
    if (filtered_weight > 7.0) {
      Serial.println("Force > 7kg → Restart");
      delay(500);
      esp_restart();
    }

    // ===== Debug =====
    Serial.print("Raw:");
    Serial.print(raw);
    Serial.print(" W:");
    Serial.print(filtered_weight, 3);
    Serial.print(" F:");
    Serial.println(filtered_force, 3);

    // ===== อัปเดต LED สถานะแบต =====
    float batteryVoltage = readBatteryVoltage();
    uint8_t battery = batteryPercent(batteryVoltage);
    updateStatusLED(batteryVoltage, battery);

    // ===== ส่ง BLE (binary packet) =====
  if (deviceConnected) {

    float safe_force =
        (filtered_force < 0.0f)
        ? 0.0f
        : filtered_force;

    int rawADC = analogRead(BATTERY_PIN);
int mv = analogReadMilliVolts(BATTERY_PIN);

Serial.print("ADC=");
Serial.print(rawADC);
Serial.print(" mV=");
Serial.println(mv);

float batteryVoltage = readBatteryVoltage();
uint8_t battery = batteryPercent(batteryVoltage);

Serial.print(" Battery: ");
Serial.print(batteryVoltage, 2);
Serial.print("V (");
Serial.print(battery);
Serial.println("%)");

    DataPacket data = {
        safe_force,
        battery
    };

    pCharacteristic->setValue(
        (uint8_t*)&data,
        sizeof(data)
    );
    pCharacteristic->notify();
}

  delay(20);
  }
}
