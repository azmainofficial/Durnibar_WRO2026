# Electrical Schematics & Pinout Mapping

> **WRO 2026 Future Engineers — Team Durnibar**

This folder contains electrical schematics, power distribution diagrams, and pinout maps for **Durnibar 2.0**.

---

## 📌 Microcontroller Pinout Table (ESP32)

| ESP32 GPIO | Connected Component | Signal Type | Description / Notes |
| :---: | :--- | :---: | :--- |
| **GPIO 21 (SDA)** | VL53L1X ToF + BNO055 IMU | I2C Data | Shared I2C bus ($400\text{ kHz}$) |
| **GPIO 22 (SCL)** | VL53L1X ToF + BNO055 IMU | I2C Clock | Shared I2C clock ($400\text{ kHz}$) |
| **GPIO 18** | Digital Steering Servo | PWM ($50\text{ Hz}$) | Steering angle pulse control |
| **GPIO 19** | ESC Motor Driver | PWM ($50\text{ Hz}$) | Rear drive motor speed control |
| **GPIO 4** | Start Button (Rule 9.11) | Digital Input | Internal pull-up, active low |
| **GPIO 16 (RX2)** | Raspberry Pi SBC UART | Serial RX | Receive velocity commands |
| **GPIO 17 (TX2)** | Raspberry Pi SBC UART | Serial TX | Send sensor telemetry |

For full electrical architecture documentation and power budget details, see **[docs/02_power_and_sensors.md](../docs/02_power_and_sensors.md)**.
