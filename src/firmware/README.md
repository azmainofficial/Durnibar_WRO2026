# ESP32 Firmware Guide — Team Durnibar WRO 2026

This directory contains the low-level C++ firmware for the **ESP32 NodeMCU** microcontroller on **Durnibar 2.0**.

---

## 📌 Features & Responsibilities

1. **Ackermann Steering Servo Control**: Servo PWM on **GPIO 18** ($50\text{ Hz}$), with safety bounds ($62^\circ$ to $118^\circ$).
2. **Brushless ESC Motor Drive Control**: ESC PWM on **GPIO 19** ($1100\mu\text{s}$ reverse to $1900\mu\text{s}$ forward, $1500\mu\text{s}$ neutral).
3. **4x Time-of-Flight (ToF) Distance Array**: Reads 4x VL53L1X distance sensors over I2C using XSHUT multiplexing on **GPIO 25, 26, 27, 14**.
4. **9-DOF IMU Orientation Tracking**: Reads BNO055 orientation sensor over I2C (**GPIO 21 SDA, GPIO 22 SCL**).
5. **WRO 2026 Rule 9.11 Start Button**: Dedicated push button on **GPIO 4** with internal pull-up and hardware debouncing.
6. **High-Speed Serial Bridge to Raspberry Pi 5**: Telemetry streaming at $50\text{ Hz}$ and command parsing over Serial2 UART (**GPIO 16 RX2, GPIO 17 TX2**) at 115200 baud.

---

## 🛠 How to Build & Flash with Arduino IDE (Primary Method)

### 1. Arduino IDE Setup
1. Open **Arduino IDE** (v2.0+ recommended).
2. Go to **File ➔ Preferences** and add the ESP32 Board Manager URL to *Additional Boards Manager URLs*:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Go to **Tools ➔ Board ➔ Boards Manager...**, search for `esp32` by Espressif Systems, and click **Install**.

### 2. Required Libraries Installation
Go to **Tools ➔ Manage Libraries...** (or `Ctrl+Shift+I`) and install:
- **`ESP32Servo`** by Kevin Harrington
- **`Adafruit MPU6050`** by Adafruit
- **`Adafruit SSD1306`** by Adafruit
- **`Adafruit GFX Library`** by Adafruit

### 3. Open & Upload Sketch
1. Open the sketch located at:
   `src/firmware/esp32_firmware/esp32_firmware.ino`
2. Connect your ESP32 board via USB.
3. Select Board: **Tools ➔ Board ➔ ESP32 Arduino ➔ ESP32 Dev Module**.
4. Select Port: **Tools ➔ Port ➔ COMx** (Windows) or `/dev/ttyUSB0` (Linux/Mac).
5. Click **Upload** (Arrow icon).

---

## 🛠 PlatformIO Setup (Alternative Method)

```bash
cd src/firmware
pio run --target upload --upload-port /dev/ttyUSB0
```

---

## 📡 Serial Telemetry & Command Protocol

### Telemetry Packet (ESP32 ➔ RPi 5 @ 50Hz)
```
$TEL,<DistFL_mm>,<DistFR_mm>,<DistSL_mm>,<DistSR_mm>,<YawAngle_deg>,<StartBtnState>#
Example: $TEL,450,448,180,185,90.5,1#
```

### Command Packet (RPi 5 ➔ ESP32)
```
$CMD,<SteeringAngle_deg>,<MotorThrottle_us>#
Example: $CMD,90,1580#
```
