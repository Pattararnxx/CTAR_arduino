#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_system.h>

// ================= BLE UUID =================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ================= I2C Pin (XIAO ESP32C6) =================
// D0 = GPIO0  -> SDA
// D8 = GPIO19 -> SCL
#define I2C_SDA 0
#define I2C_SCL 19

// ================= NAU7802 Registers =================
#define NAU_ADDR   0x2A
#define PU_CTRL    0x00
#define CTRL1      0x01
#define CTRL2      0x02
#define I2C_CTRL   0x11
#define ADCO_B2    0x12
#define PGA_REG    0x1B
#define PWR_CTRL   0x1C
#define REVISION   0x1F

// ================= BLE =================
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// ===== Battery (ยังไม่ใช้งาน เก็บไว้ทำต่อภายหลัง) =====
// const int BATTERY_PIN = 2;   // GPIO2 = A0.

// float readBatteryVoltage() {
//   uint32_t sum = 0;

//   for (int i = 0; i < 32; i++) {
//     sum += analogReadMilliVolts(BATTERY_PIN);
//     delay(2);
//   }

//   float adcVoltage = sum / 32.0f;

//   return (adcVoltage * 2.0f) / 1000.0f;
// }

// uint8_t batteryPercent(float v) {

//   if (v >= 4.20f) return 100;
//   if (v <= 3.30f) return 0;

//   const float voltageTable[] = {
//     4.20, 4.10, 4.00, 3.90,
//     3.80, 3.70, 3.60, 3.50,
//     3.30
//   };

//   const int percentTable[] = {
//     100, 90, 80, 65,
//     45, 25, 10, 5,
//     0
//   };

//   const int N =
//       sizeof(voltageTable) /
//       sizeof(float);

//   for (int i = 0; i < N - 1; i++) {

//     if (v <= voltageTable[i] &&
//         v > voltageTable[i + 1]) {

//       float v1 = voltageTable[i];
//       float v2 = voltageTable[i + 1];

//       int p1 = percentTable[i];
//       int p2 = percentTable[i + 1];

//       float percent =
//           p1 +
//           (v - v1) *
//           (p2 - p1) /
//           (v2 - v1);

//       return (uint8_t)(percent + 0.5f);
//     }
//   }

//   return 0;
// }

// ================= Calibration =================
// ปรับค่านี้ตามการ calibrate จริงของเครื่อง (หน่วย: raw count ต่อ 1 kg)
float CALIBRATION_FACTOR = 715800.0f;
int32_t zeroOffset = 0;

// ================= Filter =================
const float ALPHA = 0.2;
float filtered_weight = 0.0;
float filtered_force = 0.0;

// ================= Packet =================
struct DataPacket {
  float force;
  // uint8_t battery;
};

// =====================================================
//                 NAU7802 low-level I2C
// =====================================================
bool nauWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(NAU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t nauRead(uint8_t reg) {
  Wire.beginTransmission(NAU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);          // repeated START (สำคัญ!)
  Wire.requestFrom((uint8_t)NAU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool getBit(uint8_t reg, uint8_t bit) { return (nauRead(reg) >> bit) & 1; }

void setBit(uint8_t reg, uint8_t bit, bool v) {
  uint8_t x = nauRead(reg);
  x = v ? (x | (1 << bit)) : (x & ~(1 << bit));
  nauWrite(reg, x);
}

// อ่านผล ADC 24-bit (0x12..0x14, burst read)
int32_t readADC() {
  Wire.beginTransmission(NAU_ADDR);
  Wire.write(ADCO_B2);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)NAU_ADDR, (uint8_t)3);   // auto-increment ในตัวชิป

  uint32_t raw = 0;
  raw |= (uint32_t)Wire.read() << 16;
  raw |= (uint32_t)Wire.read() << 8;
  raw |=           Wire.read();

  // sign-extend 24-bit -> 32-bit (ค่าเป็น two's complement)
  if (raw & 0x800000) raw |= 0xFF000000;
  return (int32_t)raw;
}

// เริ่มต้นใช้งาน NAU7802
bool nauBegin() {
  // 1. Reset รีจิสเตอร์ทั้งหมด (RR = bit0)
  nauWrite(PU_CTRL, 0x01);
  delay(10);
  nauWrite(PU_CTRL, 0x00);

  // 2. Power up digital + analog (PUD=bit1, PUA=bit2)
  nauWrite(PU_CTRL, 0x06);

  // 3. รอ PUR (bit3) = 1, ปกติ ~200us
  uint32_t t0 = millis();
  while (!getBit(PU_CTRL, 3)) {
    if (millis() - t0 > 200) return false;
  }

  // 4. ใช้ LDO ภายใน (AVDDS = bit7)
  setBit(PU_CTRL, 7, true);

  // 5. CTRL1: VLDO[5:3]=100 (3.3V), GAINS[2:0]=111 (gain 128)
  nauWrite(CTRL1, (0b100 << 3) | 0b111);

  // 6. CTRL2: CRS[6:4]=011 -> 80 SPS, ช่อง 1
  nauWrite(CTRL2, 0b011 << 4);

  // 7. ค่าแนะนำจากผู้ผลิต: ปิด ADC chopper, เปิด PGA bypass cap
  nauWrite(PWR_CTRL, 0x30);
  setBit(PGA_REG, 7, true);          // PGA_CAP_EN

  // 8. Internal offset calibration (CALS = bit2 ของ CTRL2)
  nauWrite(CTRL2, (nauRead(CTRL2) & 0xF8) | 0x04);
  t0 = millis();
  while (getBit(CTRL2, 2)) {         // CALS จะกลับเป็น 0 เมื่อเสร็จ
    if (millis() - t0 > 1000) return false;
  }
  if (getBit(CTRL2, 3)) {            // CAL_ERR
    Serial.println("Calibration error!");
    return false;
  }
  return true;
}

bool nauDataReady() {
  // CR (bit5 ของ PU_CTRL) = 1 แปลว่าข้อมูลพร้อม
  return getBit(PU_CTRL, 5);
}

// เฉลี่ยค่า raw หลาย ๆ ครั้งตอนไม่มีน้ำหนักกด เพื่อหา zero offset (tare)
int32_t nauCalculateZeroOffset(uint8_t samples = 16) {
  int64_t sum = 0;
  uint8_t count = 0;
  uint32_t t0 = millis();

  while (count < samples) {
    if (nauDataReady()) {
      sum += readADC();
      count++;
    }
    if (millis() - t0 > 3000) break; // กันค้างถ้า sensor ไม่ตอบ
  }

  if (count == 0) return 0;
  return (int32_t)(sum / count);
}

// แปลง raw -> น้ำหนัก (kg) โดยใช้ zeroOffset และ CALIBRATION_FACTOR
float nauGetWeight(int32_t raw) {
  return (float)(raw - zeroOffset) / CALIBRATION_FACTOR;
}

// =====================================================
//                       BLE
// =====================================================
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

// =====================================================
//                       SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
//   pinMode(BATTERY_PIN, INPUT);
// analogReadResolution(12);
// analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  Serial.println("เริ่มระบบวัดแรง CTAR (NAU7802 raw register)...");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // เช็คว่าเจอ device บน bus ไหม
  Wire.beginTransmission(NAU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ไม่พบ NAU7802 ที่ 0x2A");
    while (1) delay(1000);
  }
  Serial.printf("Revision ID: 0x%02X\n", nauRead(REVISION) & 0x0F);

  if (!nauBegin()) {
    Serial.println("init ล้มเหลว");
    while (1) delay(1000);
  }
  Serial.println("พบ NAU7802 แล้ว");

  Serial.println("กำลัง Tare...");
  delay(500);
  zeroOffset = nauCalculateZeroOffset(16);
  Serial.print("Tare เสร็จ, zeroOffset = ");
  Serial.println(zeroOffset);

  setupBLE();
}

// =====================================================
//                        LOOP
// =====================================================
void loop() {

  if (nauDataReady()) {

    int32_t raw = readADC();
    float raw_weight = nauGetWeight(raw);

    if (raw_weight < 0.0) raw_weight = 0.0;

    // EMA Filter
    filtered_weight = (ALPHA * raw_weight) + ((1.0 - ALPHA) * filtered_weight);
    filtered_force = filtered_weight * 9.8;

    // ===== Reset เมื่อแรงเกิน =====
    if (filtered_weight > 7.0) {
      Serial.println("Force > 7kg -> Restart");
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

    // ===== ส่ง BLE (binary packet) =====
    if (deviceConnected) {

      float safe_force = (filtered_force < 0.0f) ? 0.0f : filtered_force;

//     int rawADC = analogRead(BATTERY_PIN);
// int mv = analogReadMilliVolts(BATTERY_PIN);

// Serial.print("ADC=");
// Serial.print(rawADC);
// Serial.print(" mV=");
// Serial.println(mv);

// float batteryVoltage = readBatteryVoltage();
// uint8_t battery = batteryPercent(batteryVoltage);

// Serial.print(" Battery: ");
// Serial.print(batteryVoltage, 2);
// Serial.print("V (");
// Serial.print(battery);
// Serial.println("%)");

      DataPacket data = {
          safe_force,
          // battery
      };

      pCharacteristic->setValue((uint8_t*)&data, sizeof(data));
      pCharacteristic->notify();
    }

    delay(20);
  }
}
