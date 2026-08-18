<div align="center">
  <img alt="Team Durnibar Banner" src="./assets/Banner_Durnibar.png" width="85%"/>
  <h1>🏎️ Team Durnibar — WRO 2026 Future Engineers</h1>
  <p><strong>Official Repository & Technical Documentation for World Robot Olympiad 2026 (Self-Driving Cars)</strong></p>
  <p>Representing <strong>Bangladesh 🇧🇩</strong></p>

  [![WRO Category](https://img.shields.io/badge/WRO%20Category-Future%20Engineers%202026-blue?style=for-the-badge&logo=robotics)](https://wro-association.org/)
  [![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-brightgreen?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
  [![LiDAR](https://img.shields.io/badge/LiDAR-Slamtec%20RPLIDAR%20C1-orange?style=for-the-badge)](https://www.slamtec.com/)
  [![Rules Standard](https://img.shields.io/badge/Rules%20Standard-WRO%202026%20General%20Rules-red?style=for-the-badge)](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
  [![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](./LICENSE)
</div>

---

## 📌 Table of Contents

- [1. Team Introduction](#1-team-introduction)
- [2. Mission & WRO 2026 Rules Overview](#2-mission--wro-2026-rules-overview)
- [3. Vehicle System Architecture](#3-vehicle-system-architecture)
- [4. Repository Folder Hierarchy](#4-repository-folder-hierarchy)
- [5. Mechanical & Kinematic Design](#5-mechanical--kinematic-design)
- [6. Power & Sensor Management (Wiring & Pinout)](#6-power--sensor-management-wiring--pinout)
- [7. Software Modules, Telemetry & Algorithm Strategy](#7-software-modules-telemetry--algorithm-strategy)
- [8. Hardware Components & Bill of Materials](#8-hardware-components--bill-of-materials)
- [9. Build, Calibration & Run Instructions](#9-build-calibration--run-instructions)
- [10. Autonomous Demonstration Videos](#10-autonomous-demonstration-videos)
- [11. Judging Criteria & Documentation Index](#11-judging-criteria--documentation-index)

---

## 1. Team Introduction

**Team Durnibar** is a collegiate robotics team from Bangladesh competing in the **Future Engineers** category at the **World Robot Olympiad (WRO) 2026**.

<table align="center">
  <tr>
    <td align="center" width="33%">
      <img src="./assets/team/AzmainProfile.jpg" width="160" height="160" style="border-radius: 50%;"><br><br>
      <strong>Azmain Shak Rubayed</strong><br>
      <sub>Team Lead / CAD & Vision Systems</sub><br>
      <sub>Fusion 360, ROS 2, OpenCV, Python</sub><br>
      <a href="mailto:rubayed_41220200226@nub.ac.bd">✉️ Email</a>
    </td>
    <td align="center" width="33%">
      <img src="./assets/team/rifat.jpg" width="160" height="160" style="border-radius: 50%;"><br><br>
      <strong>Rifat Ahmmed</strong><br>
      <sub>Embedded Software & Control</sub><br>
      <sub>Independent University Bangladesh</sub><br>
      <a href="mailto:ra7260352@email.com">✉️ Email</a>
    </td>
    <td align="center" width="33%">
      <img src="./assets/team/mahfuj.jpg" width="160" height="160" style="border-radius: 50%;"><br><br>
      <strong>Mahfuj Rohoman</strong><br>
      <sub>Electronics & Hardware Integration</sub><br>
      <sub>Independent University, Bangladesh</sub><br>
      <a href="mailto:member3@email.com">✉️ Email</a>
    </td>
  </tr>
</table>

---

## 2. Mission & WRO 2026 Rules Overview

The **WRO 2026 Future Engineers Self-Driving Cars** category challenges teams to engineer an autonomous 4-wheeled robotic vehicle capable of high-speed navigation, obstacle traffic sign obedience, and autonomous parallel parking on a randomly configured racetrack.

<img width="1280" height="720" alt="b3b44990-8981-4bee-be3b-6bd4b68e230d" src="https://github.com/user-attachments/assets/56936aef-776a-4c93-ac38-24e247c61853" />

### WRO 2026 Competition Challenges

#### 1. Open Challenge
- Complete **3 autonomous laps** on a track with randomly configured inner walls.
- Navigate corridors with variable widths of **600 mm or 1000 mm**.
- After completing the third lap, the robot must **autonomously stop completely within the designated finish area (with custom 15cm forward wall-avoidance adjustment)**.

#### 2. Obstacle Challenge
- Complete **3 autonomous laps** while correctly following traffic signs:
  - **Red Pillar → Keep Right**
  - **Green Pillar → Keep Left**
- Perform **autonomous parallel parking** inside a designated **20 cm-wide parking area**.

<img width="905" height="658" alt="image" src="https://github.com/user-attachments/assets/0ad2d572-e770-4012-a4b8-97f5456a3752" />

### Key Technical Rule Compliance Checklist
- **Rule 11.1 & 11.2**: Max dimensions $300 \text{ mm (L)} \times 200 \text{ mm (W)} \times 300 \text{ mm (H)}$, max weight $1.5 \text{ kg}$.
- **Rule 11.3 & 11.13**: 4-wheeled vehicle with **one single driving axle** connected via gearbox and **one steering actuator**. Differential drive (skid steering) and independent side motors are strictly prohibited.
- **Rule 9.10 & 9.11**: Vehicle power activated by **exactly one main switch**; program execution triggered by **exactly one start button**.
- **Rule 11.10**: Strictly 100% autonomous operation — no RF, Bluetooth, Wi-Fi, or remote communication during competition attempts.

---

## 3. Vehicle System Architecture

**Durnibar 2.0 ("Sulg")** uses a decoupled two-tier architecture balancing ROS 2 high-level navigation with real-time microcontroller actuator response:

```
┌─────────────────────────────────────────────────────────────────────────┐
│              HIGH-LEVEL COMPUTATION — RASPBERRY PI 5 (8GB)              │
│   • ROS 2 Humble/Jazzy Operating Framework                              │
│   • Slamtec RPLIDAR C1 (360° DTOF LaserScan Navigation & Wall Tracking) │
│   • Fifine K420 2K Webcam (108° FOV OpenCV Traffic Sign Color Detection)│
│   • High-Level ROS 2 Finite State Machine (FSM) Decision Node           │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ High-Speed UART (115200 Baud)
┌────────────────────────────────────▼────────────────────────────────────┐
│                    LOW-LEVEL CONTROLLER — ESP32 MCU                     │
│   • TB6612FNG H-Bridge Motor Driver (PWMA=13, AIN1=12, AIN2=14, STBY=27)  │
│   • Quadrature Wheel Encoder ISR (A=34, B=35) & 1D Kalman Speed Filter  │
│   • MPU6050 6-DOF IMU + QMC5883L Compass + SSD1306 OLED Display (I2C)   │
│   • Closed-Loop Steering Servo PWM (GPIO 25) & Status LEDs / Buzzer     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Repository Folder Hierarchy

```
Sulg/ (Workspace Root)
├── Sulg.ino                 # ESP32 Main Sketch Setup & Loop
├── config.h                 # ESP32 pin definitions & parameters
├── globals.h                # Global state external declarations
├── imu.cpp / imu.h          # MPU6050 & QMC5883L IMU sensor fusion
├── motor.cpp / motor.h      # TB6612 Motor driver control functions
├── encoder.cpp / encoder.h  # Hardware quadrature encoder ticks
├── odometry.cpp / .h        # Wheel-odometry displacement tracking
├── pi_comm.cpp / pi_comm.h  # Unified Raspberry Pi Serial parser & 20Hz Telemetry
├── peripherals.cpp / .h     # LEDs, Non-blocking Buzzer & debounced buttons
├── hardware_test.cpp / .h   # Diagnostic Board Self-Tests
├── calibration.cpp / .h     # Gyro, Accel, and Magnetometer EEPROM calibration
├── oled_display.cpp / .h    # SSD1306 OLED screen rendering
│
├── LICENSE                  # MIT Open Source License
├── docs/                    # Detailed Technical Documentation (Appendix C Judging Criteria)
│   ├── README.md            # Documentation Index
│   ├── 01_mobility_and_mechanics.md     # Criterion 1: Ackermann steering, gear ratio, torque calculations
│   ├── 02_power_and_sensors.md          # Criterion 2: Power budget, wiring map, ToF & IMU placement
│   ├── 03_software_and_obstacle.md      # Criterion 3: ROS 2 nodes, FSM state machine, CV, parking logic
│   ├── 04_systems_thinking_decisions.md # Criterion 4: Trade-off analysis, constraints, FMECA matrix
│   └── 05_reproducibility_guide.md      # Criterion 5: Complete BOM, step-by-step build & flash guide
│
├── journal/                 # Engineering Journal & Testing Records
│   ├── README.md            # Journal overview
│   └── testing_logs.md      # Practice lap metrics, PID tuning, parking success rates
│
├── models/                  # 3D CAD Designs & Print Files
│   ├── README.md            # Model directory index
│   ├── cad_source/          # Fusion 360 source models (Durnibar 2.0.f3z, Gear Box v2.0.f3d)
│   └── 3d_print_files/      # Printable STL / STEP files
│
├── schematics/              # Electrical Layout & Pinouts
│   ├── README.md            # ESP32 pinout table & electrical specs
│   └── wiring_diagram.png   # System power & signal wiring map
│
├── assets/                  # Images & Media
│   ├── Banner_Durnibar.png  # Team banner
│   ├── team/                # Member profile photos
│   └── vehicle/             # Vehicle photos (Front, Back, Left, Right, Top, Bottom)
│
├── videos/                  # Autonomous Driving Demonstrations
│   └── README.md            # YouTube links for Open & Obstacle Challenge (>30s runs)
│
├── ros2_ws/                 # ROS 2 Active/Staging Workspace
│   └── src/
│       ├── sulg_robot/      # Robot configurations, launch files, URDF & scripts
│       └── sllidar_ros2/    # RPLiDAR ROS 2 Driver Node
│
├── pi_code/                 # Raspberry Pi 5 Active/Staging Folder
│   ├── wro_pi_system.py     # Multithreaded Vision, RPLiDAR C1 & Web Control Server
│   ├── optimal_planner.py   # Vectorized Ackermann Optimal Trajectory & Path Optimizer (Sub-3ms)
│   ├── disparity_extender.py# Disparity Extender safety bubble & gap finder
│   ├── yolo_detector.py     # Fast HSV color segmentation & Camera undistortion
│   ├── hsv_config.json      # Color thresholds, planner weights & hardware parameters
│   ├── deploy_to_pi.py      # SSH/SCP automatic sync script
│   └── templates/index.html # Zero-lag HTML5 dashboard, Vector Radar & Trajectory Spline Ribbon
│
├── round1_open_challenge/   # Isolated Challenge 1 (Open Challenge) reactive LiDAR codebase
├── round1_open_challenge_odometry/ # Isolated Challenge 1 (Open Challenge) odometry waypoint codebase
├── round1_open_challenge_discrete/ # Isolated Challenge 1 (Open Challenge) discrete cornering state machine
└── round2_obstacle_challenge/ # Isolated Challenge 2 (Obstacle Challenge) codebase
```

---

## 5. Mechanical & Kinematic Design

Per **Criterion 1 (Mobility & Mechanical Design)**, Durnibar 2.0 uses **Ackermann Steering Kinematics** combined with a custom **24:1 two-stage spur gearbox transmission** driven by a single rear axle:

### Ackermann Steering Equation
$$\cot(\delta_o) - \cot(\delta_i) = \frac{W}{L}$$

Where $W = 145 \text{ mm}$ (track width) and $L = 175 \text{ mm}$ (wheelbase). This ensures all four wheels rotate around a single Instantaneous Center of Rotation (ICR), preventing tire slippage on narrow turns.

For complete mechanical equations, torque calculations, and CAD iteration history, see **[docs/01_mobility_and_mechanics.md](./docs/01_mobility_and_mechanics.md)**.
<img width="514" height="458" alt="image" src="https://github.com/user-attachments/assets/4cdefb3f-a21d-4510-a717-e34dddfe87f0" />

---

## 6. Power & Sensor Management (Wiring & Pinout)

Per **Criterion 2 (Power & Sensor Architecture)**, power is delivered by a **3S 11.1V 2200mAh LiPo battery** through isolated power rails:
- **High-Current Rail ($11.1\text{ V}$)**: Powers the rear brushless drive motor ESC directly.
- **Regulated Power Rail ($5.0\text{ V} / 5\text{ A}$)**: High-efficiency buck regulator supplying the Raspberry Pi 5 (8GB), ESP32 MCU, Fifine K420 webcam, ToF sensors, and IMU.

### ESP32 Pin Connections
| Component | ESP32 Pin | Description |
|---|---|---|
| **Steer Servo** | GPIO 25 | PWM steer output (SG90) |
| **TB6612 PWMA** | GPIO 13 | Motor speed PWM |
| **TB6612 AIN1** | GPIO 12 | Motor direction 1 |
| **TB6612 AIN2** | GPIO 14 | Motor direction 2 |
| **TB6612 Standby**| GPIO 27 | Active-high driver enable |
| **Encoder Channel A** | GPIO 34 | Quadrature pulse A |
| **Encoder Channel B** | GPIO 35 | Quadrature pulse B |
| **I2C SDA** | GPIO 21 | SSD1306 OLED, MPU6050 & QMC5883L |
| **I2C SCL** | GPIO 22 | I2C clock line |
| **Buzzer** | GPIO 26 | Piezo buzzer output |
| **Button 1** | GPIO 32 | Start Open Challenge (Active-LOW) |
| **Button 2** | GPIO 33 | Start Obstacle Challenge (Active-LOW) |
| **BOOT Button** | GPIO 0 | Trigger IMU Calibration (Active-LOW) |
| **LED Green** | GPIO 17 | Status indication |
| **LED Yellow** | GPIO 16 | Status indication |
| **LED Red** | GPIO 4 | Error / status indication |

### Raspberry Pi 5 Interfaces
* **USB Port 1**: Silicon Labs CP2102 USB-to-UART Bridge → **RPLiDAR C1** (460800 baud).
* **USB Port 2**: CH340 / CP2102 Serial Connection → **ESP32 Controller** (115200 baud).
* **USB Port 3**: **Fifine K420 Wide-Angle Camera** (640x368 YUYV Video Stream).

### System Power Budget Table

| Component Category | Devices | Total Nominal Power | Peak Current |
| :--- | :--- | :---: | :---: |
| **Compute & Vision** | Raspberry Pi 5 (8GB) + Fifine K420 Webcam | $10.50 \text{ W}$ | $4.00 \text{ A}$ |
| **Sensing & Logic** | ESP32 MCU + ToF + IMU | $0.83 \text{ W}$ | $0.51 \text{ A}$ |
| **Actuators & Motors** | Steering Servo + ESC Drive Motor | $18.45 \text{ W}$ | $7.30 \text{ A}$ |
| **Total System Load** | — | **$29.78 \text{ W}$** | **$11.81 \text{ A}$** |

For complete electrical schematics, pinouts, and sensor placement geometry, see **[docs/02_power_and_sensors.md](./docs/02_power_and_sensors.md)**.

---

## 7. Software Modules, Telemetry & Algorithm Strategy

Per **Criterion 3 (Software Architecture & Obstacle Strategy)**, software modules are decoupled into clean ROS 2 packages and Python nodes:

1. **Vision Module (`pi_code/yolo_detector.py`)**: Uses HSV color segmentation and adaptive thresholding on 1080p frames from the Fifine K420 webcam ($108^\circ$ FOV) to detect traffic sign pillars (Red = Pass Right, Green = Pass Left).
2. **Navigation Module (`pi_code/optimal_planner.py`)**: Computes PID steering adjustments and vectorized trajectory options to keep the vehicle centered in the track corridor.
3. **Serial Telemetry Protocol**:
The Raspberry Pi and ESP32 communicate at **115200 baud** over USB serial.
   - **ESP32 to Pi (ODOM packet @ 20 Hz)**:
     ```
     ODOM,X_mm,Y_mm,Yaw_deg,Dist_mm,Speed_mms,SensorsOK,Btn1,Btn2\n
     ```
   - **Pi to ESP32 (Control commands)**:
     * **Drive Command**: `D <speed> <steer>` (e.g. `D 130 110\n`)
       * `speed`: `-255` (full reverse) to `255` (full forward).
       * `steer`: `50` (full right) to `170` (full left). `110` is center.
     * **Stop Command**: `S\n` (sets motor to 0, centers wheels).
     * **Reset Command**: `R\n` (resets odometry coordinates and IMU reference to zero).

For complete FSM diagrams, vision pseudocode, and PID parameter tuning, see **[docs/03_software_and_obstacle.md](./docs/03_software_and_obstacle.md)**.

---

## 8. Hardware Components & Bill of Materials

Per **Criterion 5 (Reproducibility)**, below is the core hardware inventory:

| Component | Function / Role | Model / Part | Approx. Price |
| :--- | :--- | :--- | :---: |
| **Main SBC** | High-level vision & FSM | Raspberry Pi 5 (8GB RAM) | $80.00 |
| **Microcontroller** | Low-level PWM & sensor reading | ESP32 NodeMCU | $6.00 |
| **Camera** | Color traffic sign recognition | Fifine K420 2K Webcam ($108^\circ$ FOV) | $28.00 |
| **Distance Array** | Wall distance measurement | RPLiDAR C1 / ToF Sensors | $90.00 |
| **IMU** | Heading & orientation tracking | MPU6050 & QMC5883L | $8.00 |
| **Steering Servo** | Front wheel Ackermann control | Metal Gear Digital Servo | $12.00 |
| **Drive Motor** | Propulsion via 24:1 transmission | Brushless DC Motor + ESC / TB6612 | $22.00 |
| **Power Source** | Main vehicle battery | 3S 11.1V 2200mAh LiPo | $16.00 |

For full assembly guide and parts links, see **[docs/05_reproducibility_guide.md](./docs/05_reproducibility_guide.md)**.

---

## 9. Build, Calibration & Run Instructions

### 1. ESP32 Firmware & Calibration
* Rotate the bot slowly in a figure-8 motion on all axes during calibration. Offsets are automatically saved to EEPROM.
* Upload firmware to ESP32:
  ```bash
  # Flash ESP32 firmware via PlatformIO
  cd src/firmware
  pio run --target upload --upload-port /dev/ttyUSB0
  ```

### 2. Launching Raspberry Pi Control Server
Deploy the desired challenge codebase (Open Challenge or Obstacle Challenge) to the Pi, which uploads the files to the active staging directory (`~/pi_code`) on the Pi, and run the server:

**For Challenge 1 (Open Challenge):**
* **Option A: Reactive LiDAR Navigation (Default)**
  ```bash
  # Push reactive LiDAR code to Pi (deploys to ~/pi_code on Pi)
  python round1_open_challenge/deploy_to_pi.py
  
  # Run on Pi
  python3 pi_code/wro_pi_system.py
  ```
  
* **Option B: Closed-Loop Odometry Waypoint Navigation**
  ```bash
  # Push odometry waypoint code to Pi (deploys to ~/pi_code on Pi)
  python round1_open_challenge_odometry/deploy_to_pi.py
  
  # Run on Pi
  python3 pi_code/wro_pi_system.py
  ```

* **Option C: Discrete Cornering State Machine (Smooth Straight Lines)**
  ```bash
  # Push discrete state machine code to Pi (deploys to ~/pi_code on Pi)
  python round1_open_challenge_discrete/deploy_to_pi.py
  
  # Run on Pi
  python3 pi_code/wro_pi_system.py
  ```

**For Challenge 2 (Obstacle Challenge):**
```bash
# Push Challenge 2 code to Pi (deploys to ~/pi_code on Pi)
python round2_obstacle_challenge/deploy_to_pi.py

# Run on Pi
python3 pi_code/wro_pi_system.py
```
* Access the **Web Dashboard** at `http://192.168.1.101:5000` to adjust HSV color masks, set target speeds, calibrate cameras, and view live 2D lidar scans.

---

## 10. Autonomous Demonstration Videos

Per **Rule Section 7**, public YouTube video links demonstrating autonomous vehicle operation (>30 seconds per attempt) are available below:

- 📺 **[Open Challenge Autonomous Run](https://youtube.com/shorts/VsTFvdk8Iqs?si=F9PB8GRpVL1VrcOU)** — Complete 3-lap run with autonomous finish section stop.
- 📺 **[Obstacle Challenge Autonomous Run](https://youtu.be/H7GwOodIgsA)** — Complete 3-lap run with traffic sign obedience and parallel parking.

See **[videos/README.md](./videos/README.md)** for detailed video logs.

---

## 11. Judging Criteria & Documentation Index

Our technical documentation directly maps to the **WRO 2026 Appendix C Scoring Rubric (30 Points Total)**:

- 📗 **[Criterion 1: Mobility & Mechanical Design (6 pts)](./docs/01_mobility_and_mechanics.md)**
- 📘 **[Criterion 2: Power & Sensor Architecture (6 pts)](./docs/02_power_and_sensors.md)**
- 📙 **[Criterion 3: Software Architecture & Obstacle Strategy (6 pts)](./docs/03_software_and_obstacle.md)**
- 📕 **[Criterion 4: Systems Thinking & Engineering Decisions (6 pts)](./docs/04_systems_thinking_decisions.md)**
- 📓 **[Criterion 5: Reproducibility & GitHub Quality (6 pts)](./docs/05_reproducibility_guide.md)**
- 📑 **[Engineering Journal & Practice Logs](./journal/testing_logs.md)**

---

<div align="center">
  <sub>Developed with ❤️ by <strong>Team Durnibar</strong> for <strong>WRO 2026 Future Engineers</strong></sub>
</div>
