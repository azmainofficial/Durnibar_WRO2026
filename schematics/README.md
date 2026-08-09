# ESP32 Pinout Mapping & Schematics Overview

> **WRO 2026 Future Engineers — Team Durnibar**

This document specifies the exact GPIO pin mapping for the **ESP32 NodeMCU** controller on **Durnibar 2.0**.

---

## 📌 Complete ESP32 Hardware Pin Diagram

| Device / Function | ESP32 GPIO | Signal Type | Notes & Operational Rationale |
| :--- | :---: | :---: | :--- |
| **TB6612 PWMA** | `GPIO 25` | PWM Output | Motor speed control ($0 - 255$ PWM) |
| **TB6612 AIN1** | `GPIO 26` | Digital Output | Motor direction bit 1 |
| **TB6612 AIN2** | `GPIO 27` | Digital Output | Motor direction bit 2 |
| **TB6612 STBY** | `GPIO 23` | Digital Output | H-Bridge Standby / Enable pin (Driven HIGH) |
| **Encoder Phase A** | `GPIO 34` | Input Only | Quadrature encoder interrupt pin for odometry |
| **Encoder Phase B** | `GPIO 35` | Input Only | Quadrature encoder direction reference |
| **Steering Servo** | `GPIO 18` | PWM Output | Ackermann steering servo pulse ($50\text{ Hz}$) |
| **Status Buzzer** | `GPIO 19` | PWM / Tone | Audio tone feedback & system warnings |
| **LED 1 (Green)** | `GPIO 16` | Digital Output | Status LED: System Active / Autonomous Run |
| **LED 2 (Yellow)**| `GPIO 17` | Digital Output | Status LED: Booting / Warning / Calibration |
| **LED 3 (Red)**   | `GPIO 5`  | Digital Output | Status LED: Standby / Stop State |
| **Button 1**      | `GPIO 13` | Input Pull-Up | **WRO 2026 Rule 9.11 Main Start Button** |
| **Button 2**      | `GPIO 12` | Input Pull-Up | Calibration & mode toggle push button |
| **Button 3**      | `GPIO 14` | Input Pull-Up | Hardware reset / Emergency stop button |
| **OLED SDA**      | `GPIO 21` | Shared I2C | SSD1306 OLED display data ($0\times3\text{C}$) |
| **OLED SCL**      | `GPIO 22` | Shared I2C | SSD1306 OLED display clock ($400\text{ kHz}$) |
| **MPU6050 SDA**   | `GPIO 21` | Shared I2C | MPU6050 6-DOF IMU data ($0\times68$) |
| **MPU6050 SCL**   | `GPIO 22` | Shared I2C | MPU6050 6-DOF IMU clock ($400\text{ kHz}$) |
| **QMC5883L SDA**  | `GPIO 21` | Shared I2C | QMC5883L Magnetometer data ($0\times0\text{D}$) |
| **QMC5883L SCL**  | `GPIO 22` | Shared I2C | QMC5883L Magnetometer clock ($400\text{ kHz}$) |

---

## ⚡ Electrical Interface Rationale
1. **TB6612FNG H-Bridge Driver**: Driving `STBY` (GPIO 23) HIGH enables the motor outputs; setting `AIN1/AIN2` controls forward/reverse direction while `PWMA` (GPIO 25) regulates speed.
2. **Dedicated Input-Only Pins**: `GPIO 34` and `GPIO 35` are used exclusively for encoder inputs, ensuring zero conflict with output peripherals.
3. **Shared I2C Bus**: OLED display, MPU6050 IMU, and QMC5883L compass communicate over a single high-speed I2C bus (`GPIO 21` / `GPIO 22`) running at $400\text{ kHz}$.
