##  nRF52840 BLE Drone Telemetry Firmware

This firmware runs on the **Seeed XIAO nRF52840 Sense** and streams real-time orientation data to a web-based drone dashboard using Bluetooth Low Energy (BLE).

### Features

* Real-time **IMU data (LSM6DS3)** acquisition
* **Madgwick filter** for roll, pitch, yaw estimation
* BLE GATT server using **custom Nordic UART-style UUIDs**
* Live telemetry streaming to a **Chrome Web Bluetooth dashboard**
* Receives control packets (throttle, yaw, pitch, roll, arm/disarm)

###  BLE Configuration

* Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
* Characteristics:

  * Control (Write)
  * Telemetry (Notify)

###  Data Pipeline

IMU → Madgwick Filter → BLE Notify → Web Dashboard

###  Requirements

* Seeed nRF52 Arduino Core
* LSM6DS3 Library
* MadgwickAHRS Library
* Chrome browser (Web Bluetooth support)

###  Usage

1. Flash firmware to XIAO nRF52840 Sense
2. Open dashboard in Chrome
3. Connect to device `NRF_DRONE`
4. View live orientation data and send control inputs

###  Notes

* Designed for telemetry testing (not flight control)
* Yaw may drift without magnetometer calibration
* Altitude currently mocked (no barometer)

---
