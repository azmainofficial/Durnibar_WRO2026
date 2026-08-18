# WRO Autonomous Navigation & Vision System ("Sulg")

Welcome to the **Sulg** autonomous robotics navigation system built for the WRO Competition. This project integrates a **Raspberry Pi 5** (Vision, RPLiDAR C1, Sub-3ms Optimal Trajectory Planning & Web Dashboard) and an **ESP32** (IMU sensor fusion, motor driver, quadrature encoder, steering servo) over a robust, high-frequency USB serial link.

---

## 📂 Project Architecture

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
└── pi_code/                 # Raspberry Pi 5 Active/Staging Folder
    ├── wro_pi_system.py     # Multithreaded Vision, RPLiDAR C1 & Web Control Server
    ├── optimal_planner.py   # Vectorized Ackermann Optimal Trajectory & Path Optimizer (Sub-3ms)
    ├── disparity_extender.py# Disparity Extender safety bubble & gap finder
    ├── yolo_detector.py     # Fast HSV color segmentation & Camera undistortion
    ├── hsv_config.json      # Color thresholds, planner weights & hardware parameters
    ├── deploy_to_pi.py      # SSH/SCP automatic sync script
    └── templates/index.html # Zero-lag HTML5 dashboard, Vector Radar & Trajectory Spline Ribbon

├── round1_open_challenge/   # Isolated Challenge 1 (Open Challenge) reactive LiDAR codebase
├── round1_open_challenge_odometry/ # Isolated Challenge 1 (Open Challenge) odometry waypoint codebase
├── round1_open_challenge_discrete/ # Isolated Challenge 1 (Open Challenge) discrete cornering state machine
└── round2_obstacle_challenge/ # Isolated Challenge 2 (Obstacle Challenge) codebase
```

---

## ⚡ Hardware Wiring Specification

### 1. ESP32 Pin Connections
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

### 2. Raspberry Pi 5 Interfaces
* **USB Port 1**: Silicon Labs CP2102 USB-to-UART Bridge → **RPLiDAR C1** (460800 baud).
* **USB Port 2**: CH340 / CP2102 Serial Connection → **ESP32 Controller** (115200 baud).
* **USB Port 3**: **Fifine K420 Wide-Angle Camera** (640x368 YUYV Video Stream).

---

## 🔌 Serial Telemetry Protocol

The Raspberry Pi and ESP32 communicate at **115200 baud** over USB serial.

### 1. ESP32 to Pi (ODOM packet @ 20 Hz)
```
ODOM,X_mm,Y_mm,Yaw_deg,Dist_mm,Speed_mms,SensorsOK,Btn1,Btn2\n
```
* **Yaw_deg**: Gyroscope-fused heading reference ($0.0^\circ$ to $360.0^\circ$).
* **SensorsOK**: `1` if MPU6050 & Mag initialized successfully; otherwise `0`.

### 2. Pi to ESP32 (Control commands)
* **Drive Command**: `D <speed> <steer>` (e.g. `D 130 110\n`)
  * `speed`: `-255` (full reverse) to `255` (full forward).
  * `steer`: `50` (full right) to `170` (full left). `110` is center.
* **Stop Command**: `S\n` (sets motor to 0, centers wheels).
* **Reset Command**: `R\n` (resets odometry coordinates and IMU reference to zero).

---

## 🏃 Run Instructions

### 1. Calibration (ESP32)
* Hold the physical **BOOT button (GPIO 0)** down during startup to enter calibration.
* Rotate the bot slowly in a figure-8 motion on all axes. Offsets are automatically saved to EEPROM.

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
* Access the **Web Dashboard** at `http://192.168.137.137:5000` to adjust HSV color masks, set target speeds, calibrate cameras, and view live 2D lidar scans.
