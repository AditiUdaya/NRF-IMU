#include <bluefruit.h>
#include <Wire.h>
#include "LSM6DS3.h"
#include <MadgwickAHRS.h>

// ================= UUIDs (MATCH DASHBOARD) =================
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_CONTROL_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_TELEMETRY_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ================= BLE =================
BLEService droneService(SERVICE_UUID);
BLECharacteristic controlChar(CHAR_CONTROL_UUID);
BLECharacteristic telemetryChar(CHAR_TELEMETRY_UUID);

// ================= IMU =================
LSM6DS3 imu(I2C_MODE, 0x6A);
Madgwick filter;

// ================= TIMING =================
const float SAMPLE_RATE = 100.0;
const unsigned long SAMPLE_PERIOD_US = 1000000 / SAMPLE_RATE;
unsigned long lastUpdate = 0;

// ================= STATE =================
float roll, pitch, yaw;
uint8_t armed = 0;

// ================= CONTROL CALLBACK =================
void controlWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len != 8) return;

  armed = data[4];

  Serial.print("[CTRL] THR=");
  Serial.print(data[0]);
  Serial.print(" YAW=");
  Serial.print((int8_t)data[1]);
  Serial.print(" PIT=");
  Serial.print((int8_t)data[2]);
  Serial.print(" ROL=");
  Serial.print((int8_t)data[3]);
  Serial.print(" ARM=");
  Serial.println(armed);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin();

  // IMU
  if (imu.begin() != 0) {
    Serial.println(" IMU FAIL");
    while (1);
  }
  Serial.println(" IMU OK");

  filter.begin(SAMPLE_RATE);

  // BLE INIT
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("NRF_DRONE");

  // IMPORTANT: increase MTU for float data
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // ===== SERVICE + CHARS =====
  droneService.begin();

  // TELEMETRY (notify)
  telemetryChar.setProperties(CHR_PROPS_NOTIFY);
  telemetryChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  telemetryChar.setFixedLen(20);
  telemetryChar.begin();

  // CONTROL (write)
  controlChar.setProperties(CHR_PROPS_WRITE_WO_RESP);
  controlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  controlChar.setFixedLen(8);
  controlChar.setWriteCallback(controlWriteCallback);
  controlChar.begin();

  startAdvertising();

  lastUpdate = micros();

  Serial.println(" BLE READY");
}

// ================= LOOP =================
void loop() {
  unsigned long now = micros();

  if (now - lastUpdate >= SAMPLE_PERIOD_US) {
    lastUpdate = now;

    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll  = filter.getRoll();
    pitch = filter.getPitch();
    yaw   = filter.getYaw();

    sendTelemetry();
  }
}

// ================= SEND DATA =================
void sendTelemetry() {
  if (!Bluefruit.connected()) return;
  if (!telemetryChar.notifyEnabled()) return;

  uint8_t buf[20];

  memcpy(buf + 0,  &roll, 4);
  memcpy(buf + 4,  &pitch, 4);
  memcpy(buf + 8,  &yaw, 4);

  float altitude = 0;
  memcpy(buf + 12, &altitude, 4);

  buf[16] = 0;       // mode
  buf[17] = armed;   // arm state

  uint16_t timeSec = millis() / 1000;
  memcpy(buf + 18, &timeSec, 2);

  telemetryChar.notify(buf, 20);
}

// ================= ADVERTISING =================
void startAdvertising() {
  Bluefruit.Advertising.stop();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  //  CRITICAL FIX
  Bluefruit.Advertising.addService(droneService);

  Bluefruit.Advertising.addName();

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);

  Bluefruit.Advertising.start(0);
}
