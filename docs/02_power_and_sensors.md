# Criterion 2: Power and Sensor Architecture

> **WRO 2026 Future Engineers — Technical Report**  
> **Team Durnibar | Bangladesh**

---

## 1. Electrical Architecture Overview

The electrical system of **Durnibar 2.0** is divided into two isolated power domains to eliminate ground loops and voltage dips caused by motor startup surges:
1. **High-Current Drive Domain**: Powers the main drive motor and steering servo directly through dedicated motor drivers.
2. **Low-Noise Compute & Sensing Domain**: Powers the single-board computer (SBC), microcontroller unit (MCU), camera, ToF distance sensors, and IMU via a high-efficiency buck regulator.

---

## 2. Power Budget & Current Draw Calculation

All power is supplied by a single **3S LiPo Battery** ($11.1 \text{ V}$ nominal, $12.6 \text{ V}$ fully charged, $2200 \text{ mAh}$, $35\text{C}$ discharge rating).

### Power Consumption Breakdown

| Component | Voltage | Nominal Current | Peak Current | Power (Nominal) |
| :--- | :--- | :--- | :--- | :--- |
| **Main SBC (Raspberry Pi 5 8GB)** | $5.0 \text{ V}$ | $1.80 \text{ A}$ | $3.50 \text{ A}$ | $9.00 \text{ W}$ |
| **RPLIDAR C1 DTOF 360° LiDAR** | $5.0 \text{ V}$ | $0.50 \text{ A}$ | $0.80 \text{ A}$ | $2.50 \text{ W}$ |
| **MCU Board (ESP32 NodeMCU)** | $3.3 \text{ V}$ | $0.15 \text{ A}$ | $0.30 \text{ A}$ | $0.50 \text{ W}$ |
| **Fifine K420 Webcam (108° FOV)** | $5.0 \text{ V}$ | $0.30 \text{ A}$ | $0.50 \text{ A}$ | $1.50 \text{ W}$ |
| **ToF Distance Sensors (4x VL53L1X)**| $3.3 \text{ V}$| $0.08 \text{ A}$ | $0.16 \text{ A}$ | $0.26 \text{ W}$ |
| **IMU (MPU6050 6-DOF)** | $3.3 \text{ V}$ | $0.02 \text{ A}$ | $0.05 \text{ A}$ | $0.07 \text{ W}$ |
| **Steering Servo (Metal Gear)** | $6.0 \text{ V}$ | $0.30 \text{ A}$ | $1.80 \text{ A}$ | $1.80 \text{ W}$ |
| **Brushless Drive Motor + ESC** | $11.1 \text{ V}$ | $1.50 \text{ A}$ | $5.50 \text{ A}$ | $16.65 \text{ W}$ |
| **Total System Requirements** | — | **$4.65 \text{ A}$** | **$12.61 \text{ A}$** | **$32.28 \text{ W}$** |

### Battery Runtime Estimation

To preserve cell health and avoid over-discharge, we apply an **80% Depth of Discharge (DoD)** safety margin to the calculations:

#### Option A: 3S 1000 mAh (1.0 Ah) Battery
$$T_{\text{runtime\_safe}} = \frac{\text{Capacity (Ah)} \times 0.80}{\text{Nominal Current (A)}} = \frac{1.0 \text{ Ah} \times 0.80}{4.65 \text{ A}} \approx 0.172 \text{ hours} \approx 10.3 \text{ minutes}$$
*With a 3-minute competition run time, a single charge provides capacity for **~3 full runs**.*

#### Option B: 3S 2200 mAh (2.2 Ah) Battery
$$T_{\text{runtime\_safe}} = \frac{\text{Capacity (Ah)} \times 0.80}{\text{Nominal Current (A)}} = \frac{2.2 \text{ Ah} \times 0.80}{4.65 \text{ A}} \approx 0.378 \text{ hours} \approx 22.7 \text{ minutes}$$
*Provides capacity for **~7 full runs** on a single charge.*

---

## 3. Sensor Suite & Spatial Geometry

```
                    FRONT OF VEHICLE
         ┌───────────────────────────────────┐
         │[RPLIDAR C1] [Fifine K420] [ToF-FR]│
         │         \        │        /       │
         │          \       │       /        │
         │           \      │      /         │
LEFT     │ [ToF-SideL]  [MPU6050]  [ToF-SideR]│     RIGHT
         │                  │                │
         │             [Encoders]            │
         └───────────────────────────────────┘
                    REAR OF VEHICLE
```

### Sensor Specifications & Placement Rationale

1. **Slamtec RPLIDAR C1 (Top Center, 360° DTOF Laser Scanner)**:
   - **Role**: 360-degree high-precision distance scanning ($12\text{ m}$ range, $10\text{ Hz}$ scan rate) for track boundary mapping, wall distance feedback, and parallel parking lot detection.
   - **ROS 2 Integration**: Publishes `sensor_msgs/msg/LaserScan` on `/scan`.

2. **Fifine K420 Webcam (Front Center, $108^\circ$ FOV, $15^\circ$ Downward Tilt)**:
   - **Role**: High-definition (1080p @ 30 FPS) color traffic sign detection (Red vs Green pillars).
   - **FOV Coverage**: $108^\circ$ diagonal field of view provides broad lane coverage ($>75\text{ cm}$ span at $35\text{ cm}$ forward projection), capturing both inner boundary walls and traffic pillars simultaneously without image distortion.

2. **ToF Distance Sensors (VL53L1X x4)**:
   - **Front-Left & Front-Right ($30^\circ$ Angled Outward)**: Early detection of outer track walls and pillars.
   - **Side-Left & Side-Right ($90^\circ$ Perpendicular)**: Wall-following distance reference ($600\text{ mm} - 1000\text{ mm}$ corridor tracking) and parallel parking depth estimation.

3. **Inertial Measurement Unit (IMU - BNO055 / MPU6050)**:
   - **Role**: 9-DOF Absolute orientation tracking for turn detection ($90^\circ$ cornering) and straight-line drift compensation.

4. **Hall Effect Optical Wheel Encoders**:
   - **Role**: Odometry feedback measuring precise linear distance traveled per wheel rotation (resolution: 480 pulses per revolution).

---

## 4. System Wiring Map & Pinout Specification

```
[ 3S LiPo 11.1V ] ───► [ Main Power Switch (Rule 9.10) ]
                                │
        ┌───────────────────────┴───────────────────────┐
        ▼                                               ▼
[ Buck Regulator 5V 5A ]                       [ TB6612FNG Driver ]
        │                                               │
        ├───► Raspberry Pi / SBC ──(USB)──► Camera      ├───► (PWMA: GPIO 25, AIN1/2: GPIO 26/27, STBY: GPIO 23)
        │                                               └───► Drive Motor
        └───► Microcontroller (ESP32)
                    │
                    ├───(I2C SDA: GPIO 21, SCL: GPIO 22)──► ToF Sensors (VL53L1X, 0x29)
                    ├───(I2C SDA: GPIO 21, SCL: GPIO 22)──► IMU Sensor (MPU6050, 0x68 / BNO055, 0x28)
                    ├───(PWM 50Hz: GPIO 18)───────────────► Steering Servo (MG92B)
                    └───(GPIO: GPIO 13, Pull-up)──────────► Start Button (Rule 9.11)
```

---

## 5. Calibration & Noise Mitigation

- **Optical Noise**: Software adaptive white balance and dynamic HSV thresholding compensate for varying venue lighting conditions.
- **Vibration Isolation**: Silicone dampening pads isolate the IMU module from high-frequency chassis vibrations generated by the gearbox.
- **ToF Filtering**: Moving-average filter ($N=5$ sample window) removes outlier distance spikes.
