#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>
#include <cstring>

NAU7802 myScale;

// ===== UUID =====
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ===== I2C =====
const int I2C_SDA = 20;
const int I2C_SCL = 18;

// ===== Battery / active-low built-in LED =====
constexpr uint8_t BATTERY_PIN = A1;
constexpr float DIVIDER_RATIO = 2.02f;
constexpr float CALIBRATION_GAIN = 1.0f;
constexpr uint16_t SAMPLE_COUNT = 128;
constexpr float BATTERY_EMPTY_V = 3.2f;
constexpr float BATTERY_FULL_V = 3.92f;
#define STATUS_LED LED_BUILTIN
bool batteryReady = false;
uint8_t latestBatteryPercent = 0;
// ===== BLE =====
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
std::atomic<bool> deviceConnected{false};
bool bleReady = false;

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


// Voltage-based estimate: 3.20 V = 0%, 3.92 V = 100%.
// This is a linear display scale, not a measured state of charge.
uint8_t batteryPercent(float v) {
  if (v <= BATTERY_EMPTY_V) return 0;
  if (v >= BATTERY_FULL_V) return 100;
  return uint8_t((v - BATTERY_EMPTY_V) * 100.0f /
                 (BATTERY_FULL_V - BATTERY_EMPTY_V) + 0.5f);
}
enum class LedMode { Off, On, Blink, FastBlink };
LedMode ledMode = LedMode::Off;
bool ledOn = false;
uint32_t lastToggleMs = 0;

void setBatteryLed(float batteryV, uint32_t now) {
  LedMode next;
  if (batteryV > 3.96f || batteryV < 2.5f) next = LedMode::Off;
  else if (batteryV >= 3.65f) next = LedMode::On;
  else if (batteryV < 3.3f) next = LedMode::FastBlink;
  else next = LedMode::Blink;

  if (next != ledMode) {
    ledMode = next;
    ledOn = next != LedMode::Off;
    lastToggleMs = now;
  }
}

void serviceLed(uint32_t now) {
  if (ledMode == LedMode::Off) ledOn = false;
  else if (ledMode == LedMode::On) ledOn = true;
  else {
    const uint32_t intervalMs = ledMode == LedMode::FastBlink ? 100 : 500;
    if (uint32_t(now - lastToggleMs) >= intervalMs) {
      lastToggleMs = now;
      ledOn = !ledOn;
    }
  }
  digitalWrite(LED_BUILTIN, ledOn ? LOW : HIGH);
}

// Collect the same 128 samples without blocking the fast LED blink.
void serviceBattery(uint32_t now) {
  static uint32_t lastSampleMs = 0;
  static uint32_t lastBatchMs = 0;
  static uint32_t sumMv = 0;
  static uint16_t count = 0;
  static bool firstBatch = true;

  if (uint32_t(now - lastSampleMs) < 2) return;
  if (count == 0) {
    if (!firstBatch && uint32_t(now - lastBatchMs) < 1000) return;
    firstBatch = false;
    lastBatchMs = now;
  }
  lastSampleMs = now;
  sumMv += analogReadMilliVolts(BATTERY_PIN);
  if (++count == SAMPLE_COUNT) {
    const float batteryV = (sumMv / float(SAMPLE_COUNT)) *
                           DIVIDER_RATIO * CALIBRATION_GAIN / 1000.0f;
    if (!batteryReady) filteredVoltage = batteryV;
    else filteredVoltage += VOLTAGE_ALPHA * (batteryV - filteredVoltage);
    batteryReady = true;
    latestBatteryPercent = batteryPercent(filteredVoltage);
    setBatteryLed(batteryV, now);
    Serial.printf("Battery: %.3f V\n", batteryV);
    sumMv = 0;
    count = 0;
  }
}

// ===== Packet (original BLE format) =====
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

  // Keep the original struct size/offsets, with deterministic padding.
  DataPacket initial;
  memset(&initial, 0, sizeof(initial));
  initial.battery = latestBatteryPercent;
  pCharacteristic->setValue((uint8_t*)&initial, sizeof(initial));
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



// ===== Cooperative scale startup and sampling =====
// Same NAU7802 configuration as the installed SparkFun begin(), but waits
// are split across loop iterations so battery and LED continue running.
enum class ScaleState { Retry, ResetWait, PowerWait, LdoWait, Flush,
                        Calibrating, Settle, Tare, Ready };
ScaleState scaleState = ScaleState::Retry;
uint32_t scaleSince = 0;
uint32_t lastScalePoll = 0;
uint32_t lastReadingMs = 0;
bool firstScaleAttempt = true;
bool zeroReady = false;
bool overloaded = false;
int32_t savedZero = 0;
int64_t scaleSum = 0;
uint8_t scaleCount = 0;
constexpr uint8_t SCALE_SAMPLES = 8;
constexpr uint8_t SCALE_ADDRESS = 0x2A;

// The library returns a numeric value even on some I2C errors.
// Check complete transfers before treating a conversion as valid.
bool readScaleBytes(uint8_t reg, uint8_t* bytes, uint8_t length) {
  Wire.beginTransmission(SCALE_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(SCALE_ADDRESS, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) bytes[i] = Wire.read();
  return true;
}

void scaleFailure(uint32_t now) {
  scaleState = ScaleState::Retry;
  scaleSince = now;
  scaleSum = 0;
  scaleCount = 0;
  // Preserve savedZero across reconnects; never tare a loaded sensor again.
  Serial.println("Scale unavailable; retry in 3 s. Battery/LED remain active.");
}

void publishForce(int32_t rawAverage) {
  float weight = float(int64_t(rawAverage) - savedZero) / CALIBRATION_FACTOR;
  if (weight < 0.0f) weight = 0.0f;
  filtered_weight += ALPHA * (weight - filtered_weight);
  filtered_force = filtered_weight * 9.8f;

  const bool nextOverload = filtered_weight > 10.0f;
  if (nextOverload != overloaded) {
    overloaded = nextOverload;
    Serial.println(overloaded ? "OVERLOAD > 10 kg: release load" : "Overload cleared");
  }
  // Continue reporting measured force during overload. No restart or re-tare.
  Serial.printf("Raw:%ld W:%.3f F:%.3f\n",
                (long)rawAverage, filtered_weight, filtered_force);

  if (bleReady && batteryReady) {
    DataPacket data;
    memset(&data, 0, sizeof(data));
    data.force = filtered_force;
    data.battery = latestBatteryPercent;
    pCharacteristic->setValue((uint8_t*)&data, sizeof(data));
    if (deviceConnected.load()) pCharacteristic->notify();
  }
}

void serviceScale(uint32_t now) {
  if (scaleState == ScaleState::Retry) {
    if (!firstScaleAttempt && uint32_t(now - scaleSince) < 3000) return;
    firstScaleAttempt = false;
    // Bind the Wire port without the library's blocking initialization.
    if (!myScale.begin(Wire, false) ||
        !myScale.setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)) {
      scaleFailure(now);
      return;
    }
    scaleState = ScaleState::ResetWait;
    scaleSince = now;
    return;
  }

  if (uint32_t(now - lastScalePoll) < 2) return;
  lastScalePoll = now;
  uint8_t control = 0;
  if (!readScaleBytes(NAU7802_PU_CTRL, &control, 1)) {
    scaleFailure(now);
    return;
  }

  if (scaleState == ScaleState::ResetWait) {
    if (uint32_t(now - scaleSince) < 2) return;
    if (!myScale.clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL) ||
        !myScale.setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL) ||
        !myScale.setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL)) {
      scaleFailure(now);
      return;
    }
    scaleState = ScaleState::PowerWait;
    scaleSince = now;
    return;
  }

  if (scaleState == ScaleState::PowerWait) {
    if (!(control & (1u << NAU7802_PU_CTRL_PUR))) {
      if (uint32_t(now - scaleSince) > 100) scaleFailure(now);
      return;
    }
    uint8_t adc = 0;
    if (!myScale.setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL) ||
        !myScale.setLDO(NAU7802_LDO_3V3) ||
        !myScale.setGain(NAU7802_GAIN_128) ||
        !myScale.setSampleRate(NAU7802_SPS_80) ||
        !readScaleBytes(NAU7802_ADC, &adc, 1) ||
        !myScale.setRegister(NAU7802_ADC, adc | 0x30) ||
        !myScale.setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR) ||
        !myScale.clearBit(NAU7802_PGA_LDOMODE, NAU7802_PGA)) {
      scaleFailure(now);
      return;
    }
    myScale.setCalibrationFactor(CALIBRATION_FACTOR);
    scaleState = ScaleState::LdoWait;
    scaleSince = now;
    return;
  }

  if (scaleState == ScaleState::LdoWait) {
    if (uint32_t(now - scaleSince) < myScale.getLDORampDelay()) return;
    scaleState = ScaleState::Flush;
    scaleCount = 0;
    lastReadingMs = now;
    return;
  }

  if (scaleState == ScaleState::Calibrating) {
    uint8_t cal = 0;
    if (!readScaleBytes(NAU7802_CTRL2, &cal, 1)) {
      scaleFailure(now);
      return;
    }
    if (cal & (1u << NAU7802_CTRL2_CALS)) {
      if (uint32_t(now - scaleSince) > 1000) scaleFailure(now);
      return;
    }
    if (cal & (1u << NAU7802_CTRL2_CAL_ERROR)) {
      scaleFailure(now);
      return;
    }
    scaleState = ScaleState::Settle;
    scaleSince = now;
    Serial.println(zeroReady ? "Scale recovered; keeping zero" :
                              "Tare starting: keep the scale unloaded");
    return;
  }

  if (scaleState == ScaleState::Settle) {
    if (uint32_t(now - scaleSince) < 1000) return;
    scaleState = zeroReady ? ScaleState::Ready : ScaleState::Tare;
    scaleSum = 0;
    scaleCount = 0;
    lastReadingMs = now;
    // Discard the conversion held from the settling period.
    uint8_t discard[3];
    if (!readScaleBytes(NAU7802_ADCO_B2, discard, 3)) scaleFailure(now);
    return;
  }

  if (!(control & (1u << NAU7802_PU_CTRL_CR))) {
    if (uint32_t(now - lastReadingMs) > 1000) scaleFailure(now);
    return;
  }
  uint8_t bytes[3];
  if (!readScaleBytes(NAU7802_ADCO_B2, bytes, 3)) {
    scaleFailure(now);
    return;
  }
  int32_t raw = (uint32_t(bytes[0]) << 16) | (uint32_t(bytes[1]) << 8) | bytes[2];
  if (raw & 0x800000) raw -= 0x1000000;
  lastReadingMs = now;

  if (scaleState == ScaleState::Flush) {
    if (++scaleCount == 10) {
      uint8_t cal = 0;
      if (!readScaleBytes(NAU7802_CTRL2, &cal, 1) ||
          !myScale.setRegister(NAU7802_CTRL2,
                             (cal & 0xFC) | (1u << NAU7802_CTRL2_CALS))) {
        scaleFailure(now);
        return;
      }
      scaleState = ScaleState::Calibrating;
      scaleSince = now;
      scaleCount = 0;
    }
    return;
  }

  scaleSum += raw;
  if (++scaleCount < SCALE_SAMPLES) return;
  const int32_t average = int32_t(scaleSum / SCALE_SAMPLES);
  scaleSum = 0;
  scaleCount = 0;
  if (scaleState == ScaleState::Tare) {
    savedZero = average;
    myScale.setZeroOffset(savedZero);
    zeroReady = true;
    scaleState = ScaleState::Ready;
    Serial.println("Tare complete; scale ready");
    return;
  }
  publishForce(average);
}

void setup() {
  Serial.begin(115200);
  digitalWrite(STATUS_LED, HIGH);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(10); // Bound a failed I2C transaction.
  Serial.println("CTAR starting: keep the scale unloaded for initial tare");
}

void loop() {
  serviceBattery(millis());
  serviceLed(millis());
  // Advertising starts only after the first completed battery measurement.
  if (batteryReady && !bleReady) {
    setupBLE();
    bleReady = true;
  }
  serviceScale(millis());
  serviceLed(millis());
  delay(1);
}