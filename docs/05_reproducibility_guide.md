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

### Electronics & Sensors

| Item Name | Quantity | Specifications | Approx. Unit Price (USD) | Function / Notes |
| :--- | :---: | :--- | :---: | :--- |
| **Raspberry Pi 5 (8GB RAM)** | 1 | Quad-core 2.4GHz ARM Cortex-A76 | $80.00 | High-level ROS 2 compute & vision processing |
| **RPLIDAR C1 DTOF 360° LiDAR** | 1 | $12\text{m}$ Range, $10\text{Hz}$, 460800 Baud | $79.00 | 360° LaserScan navigation & wall detection |
| **ESP32 NodeMCU MCU** | 1 | Dual-core 240MHz MCU | $6.00 | Low-level motor PWM & encoder ISR |
| **Fifine K420 2K Webcam** | 1 | 1080p @ 30FPS, $108^\circ$ FOV | $28.00 | Color traffic sign & pillar detection |
| **MPU6050 6-DOF IMU** | 1 | Gyro & Accelerometer (I2C) | $3.50 | Yaw rate & acceleration tracking |
| **Quadrature Wheel Encoder** | 1 | Phase A/B 480 Ticks/Rev | $8.00 | Real-time wheel odometry ticks |
| **Digital Steering Servo** | 1 | $6.0\text{ V}, 4.8\text{ kg}\cdot\text{cm}$ | $12.00 | Front wheel Ackermann steering control |
| **TB6612FNG Motor Driver** | 1 | Dual H-Bridge (Bridged Mode) | $4.50 | Rear wheel motor drive control |
| **3S LiPo Battery** | 1 | $11.1\text{ V}, 2200\text{ mAh}$ | $16.00 | Main system power source |
| **5V 5A Buck Regulator** | 1 | High-Efficiency DC-DC (Pi 5 & LiDAR) | $6.00 | Logic & SBC power supply |

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
