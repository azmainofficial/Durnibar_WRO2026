# Criterion 5: Reproducibility and Build Guide

> **WRO 2026 Future Engineers — Technical Report**  
> **Team Durnibar | Bangladesh**

---

## 1. Overview & Reproducibility Goal

This guide provides step-by-step instructions, complete **Bill of Materials (BOM)**, CAD models, wiring maps, and compilation scripts necessary to reproduce **Team Durnibar's** self-driving robot from scratch.

---

## 2. Complete Bill of Materials (BOM)

### Mechanical & Hardware Components

| Item Name | Quantity | Manufacturer / Source | Approx. Unit Price (USD) | Function / Notes |
| :--- | :---: | :--- | :---: | :--- |
| **Custom Carbon Fiber Chassis** | 1 | Custom CNC Cut (3mm) | $18.00 | Lower deck chassis frame |
| **3D Printed PETG Upper Deck** | 1 | PETG Filament | $4.00 | Upper deck sensor & compute mount |
| **Rubber Tires ($65\text{ mm}$)** | 4 | Tamiya / RC Hobby | $8.00 | High-grip rubber tires |
| **Ackermann Steering Knuckles**| 2 | Aluminium / 3D Printed | $6.00 | Front wheel pivots |
| **Metal Ball-Joint Tie Rods** | 2 | M3 RC Tie Rods | $4.00 | Zero-backlash steering linkage |
| **24:1 2-Stage Spur Gear Set** | 1 | Nylon / 3D Printed | $5.00 | Rear drive transmission |

## Electronics & Sensors

> Hardware BOM and sourcing information for the robot's compute, sensing, control, and power systems.

### Component & Pricing

| # | Component | Qty | Key Specification | 🇧🇩 Bangladesh Source | 🇧🇩 Price | 🌎 International Source | 🌎 Price | Status |
| :-: | :--- | :-: | :--- | :--- | ---: | :--- | ---: | :--- |
| 01 | **Raspberry Pi 5 (8GB RAM)** | 1 | Quad-core 2.4GHz ARM Cortex-A76 | [RoboticsBD](https://store.roboticsbd.com/raspberry-pi/2439-raspberry-pi-5-8gb-robotics-bangladesh.html) | **৳15,945** | [Raspberry Pi](https://www.raspberrypi.com/products/raspberry-pi-5/) | **~$125** | Exact Match |
| 02 | **RPLIDAR C1 DTOF 360° LiDAR** | 1 | 12m range, 360°, 10Hz typical, 460800 baud | [RoboticsBD](https://store.roboticsbd.com/sensors/2933-rplidar-c1-dtof-lidar-360-laser-range-scanner-12m-ip54-robotics-bangladesh.html) | **৳13,900** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=SLAMTEC+RPLIDAR+C1) | **~$70–90** | Exact Match |
| 03 | **ESP32 NodeMCU MCU** | 1 | Dual-core ESP32, 240MHz, 3.3V logic | [RoboticsBD](https://store.roboticsbd.com/development-boards/2268-esp32-v13-dev-board-ch340c-nodemcu-32-robotics-bangladesh.html) | **৳580** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=ESP32+NodeMCU+DevKit+V1) | **~$3–7** | Compatible Equivalent |
| 04 | **FIFINE K420 2K Webcam** | 1 | 1080p @ 30FPS, 108° FOV | [FIFINE Bangladesh](https://www.fifine-bd.com/fifine-k420-2k-computer-webcam) | **~৳3,290** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=FIFINE+K420+2K+webcam) | **~$25–35** | Exact Match |
| 05 | **MPU6050 6-DOF IMU** | 1 | 3–5V, Gyroscope + Accelerometer, I2C | [RoboticsBD](https://store.roboticsbd.com/robotics-parts/104-6dof-accelerometer-gyroscope-gy-521-mpu-6050-robotics-bangladesh.html) | **৳360** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=GY-521+MPU6050) | **~$1.50–4** | Exact Match |
| 06 | **Quadrature Wheel Encoder** | 1 | Phase A/B, 480 ticks/rev required | [RoboticsBD — 600 P/R Encoder](https://store.roboticsbd.com/speed-detection-sensor-robotics-bangladesh/3322-hn3806-photoelectric-rotary-encoder-600-pr-2-phase-robotics-bangladesh.html) | **Verify current listing** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=600+PPR+quadrature+encoder) | **~$5–15** | Compatible Equivalent — 600 P/R |
| 07 | **Digital Steering Servo** | 1 | 6.0V, ≥4.8kg·cm, digital servo | [RoboticsBD — MG995](https://store.roboticsbd.com/motor/278-servo-motor-mg995-180-degree-rotation-robotics-bangladesh.html) | **~৳350–800** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=MG995+digital+servo+10kg) | **~$4–10** | Compatible Equivalent — higher torque |
| 08 | **TB6612FNG Motor Driver** | 1 | Dual H-Bridge, up to 1.2A continuous/channel | [RoboticsBD](https://store.roboticsbd.com/robotics-parts/684-motor-driver-dual-tb6612fng-1a-robotics-bangladesh.html) | **৳219** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=TB6612FNG+motor+driver+module) | **~$1.50–4** | Exact Match |
| 09 | **3S LiPo Battery** | 1 | 11.1V, 2200mAh, 3S | [RoboticsBD](https://store.roboticsbd.com/battery/930-lipo-battery-2200mah-111v-3s-robotics-bangladesh.html) | **৳2,500** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=11.1V+2200mAh+3S+LiPo+XT60) | **~$12–20** | Exact Match |
| 10 | **5V 5A Buck Regulator** | 1 | DC-DC, 9–36V input, 5V/5A output | [RoboticsBD — LM2596S 5V 5A](https://store.roboticsbd.com/power-module-adapter/2222-lm2596s-dc-dc-24v12v-to-5v-5a-step-down-power-supply-buck-converter-charging-module-robotics-bangladesh.html) | **৳380** | [AliExpress](https://www.aliexpress.com/wholesale?SearchText=12V+24V+to+5V+5A+buck+converter) | **~$2.50–6** | Compatible Equivalent |

---

## 3. Hardware Assembly Step-by-Step

```
Step 1: Lower Chassis Assembly
├── Mount rear axle bearings to carbon fiber lower deck.
├── Install 24:1 gearbox transmission (`models/cad_source/Gear Box v2.0.f3d`).
└── Attach front Ackermann steering knuckles and digital steering servo.

Step 2: Electronics & Wiring Integration
├── Secure 3S LiPo battery in lower chassis compartment.
├── Mount main power switch on vehicle exterior (Rule 9.10).
├── Wire ESC and steering servo to lower power distribution board.
└── Route I2C lines from 4x VL53L1X ToF sensors to ESP32 MCU.

Step 3: Upper Deck & Compute Integration
├── Attach PETG upper deck platform using M3 standoffs.
├── Mount Raspberry Pi 4B and wide-angle USB camera ($15^\circ$ tilt).
└── Connect serial UART bridge between Raspberry Pi and ESP32 MCU.
```

---

## 4. Software Compilation & Flashing Instructions

### Prerequisites
- **Ubuntu 22.04 LTS** (or Raspberry Pi OS 64-bit)
- **ROS 2 Humble Hawksbill**
- **Python 3.10+** with `opencv-python`, `numpy`, `pyyaml`
- **PlatformIO / Arduino IDE** (for ESP32 MCU firmware)

### Building the ROS 2 High-Level Stack

```bash
# 1. Clone repository to workspace
cd ~/ros2_ws/src
git clone https://github.com/azmainofficial/Durnibar_WRO2026.git

# 2. Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Build ROS 2 workspace
colcon build --symlink-install --packages-select durnibar_vision durnibar_nav durnibar_fsm

# 4. Source setup file
source install/setup.bash
```

### Flashing Microcontroller Firmware (ESP32)

```bash
# Navigate to firmware directory
cd src/firmware

# Compile and flash via PlatformIO
pio run --target upload --upload-port /dev/ttyUSB0
```

---

## 5. Execution Procedure (Starting Round - Rules 9.10 & 9.11)

1. Place the vehicle completely inside the starting zone switched OFF (Rule 9.6).
2. Flip the single **Main Power Switch** to turn on vehicle electronics (Rule 9.10).
3. The vehicle enters `BOOT_WAIT` state, awaiting the start button.
4. On judge's "GO!" signal, press the single **Start Push Button** (Rule 9.11).
5. The robot begins autonomous execution immediately.
