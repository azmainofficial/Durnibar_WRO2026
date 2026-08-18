# Criterion 4: Systems Thinking and Engineering Decisions

> **WRO 2026 Future Engineers — Technical Report**  
> **Team Durnibar | Bangladesh**

---

## 1. Systems Thinking Framework

In engineering **Durnibar 2.0**, every component decision was evaluated based on its ripple effects across all physical and software subsystems:

```
                  ┌──────────────────────────────┐
                  │    COMPUTATION & VISION      │
                  │ Frame rate, Latency, CPU %   │
                  └──────────────┬───────────────┘
                                 │
         ┌───────────────────────┴───────────────────────┐
         ▼                                               ▼
┌──────────────────────────────┐               ┌──────────────────────────────┐
│     MECHANICAL & KINEMATICS  │               │      POWER & ELECTRONICS     │
│ Weight, Ackermann, Center    │◄─────────────►│ Current draw, Regulation,    │
│ of Gravity (CG), Friction    │               │ Battery runtime, Noise       │
└──────────────────────────────┘               └──────────────────────────────┘
```

---

## 2. Quantitative Trade-off Analysis ("We Chose X Instead of Y")

| Decision Domain | Option Considered (Y) | Selected Solution (X) | Quantitative Justification & Trade-off Reasoning |
| :--- | :--- | :--- | :--- |
| **Drive Kinematics** | Differential Drive (2 Motors) | **Ackermann Steering + Single Rear Drive** | Differential drive allows zero-radius turns, but is **explicitly prohibited** under Rule 11.3 & 11.5. Ackermann steering complies 100% with rules and offers superior high-speed directional stability. |
| **Vision Processor** | Edge Cloud Vision (Offboard) | **Onboard Raspberry Pi 5 (8GB) + Fifine K420 (108° FOV)** | Offboard processing reduces vehicle weight, but introduces wireless latency ($>120\text{ ms}$) and violates Rule 11.10 (no wireless communication allowed during rounds). Onboard RPi 5 (8GB) processes full 1080p @ 30 FPS vision streams with latency $<15\text{ ms}$. |
| **Frame Rate vs CPU Load** | $60\text{ FPS}$ Vision Stream | **$30\text{ FPS}$ Vision + Downsampled 480p** | Running 60 FPS pushed CPU temperatures above $78^\circ\text{C}$ and induced thermal throttling. Reducing to 30 FPS at 480p kept CPU load under $55\%$ and temperature $<52^\circ\text{C}$ with negligible impact on detection accuracy. |
| **Gear Reduction Ratio** | High-Speed 1:12 Gearbox | **High-Torque 1:24 Gearbox** | 1:12 ratio achieved top speed of $2.2\text{ m/s}$, but caused jerky low-speed control during parallel parking. 1:24 ratio lowered top speed to $1.16\text{ m/s}$ while increasing parking success rate from $45\%$ to $95\%$. |

---

## 3. Failure Modes, Effects, and Criticality Analysis (FMECA)

```
+-----------------------------------------------------------------------------------+
|                            FAILURE MITIGATION MATRIX                              |
+--------------------------+-----------------------+--------------------------------+
| Potential Failure        | Risk Impact           | Implemented Software/Hardware  |
|                          |                       | Safeguard                      |
+--------------------------+-----------------------+--------------------------------+
| Camera Glare / Overexposure| High (Missed traffic  | Adaptive dynamic HSV          |
|                          | sign side obedience)  | windowing + color saturation   |
|                          |                       | validation                     |
+--------------------------+-----------------------+--------------------------------+
| ToF Sensor Glitch /      | Medium (Erroneous wall| Moving-average median filter   |
| Disconnection            | proximity readings)   | (N=5 window) + fallback to     |
|                          |                       | IMU straight-line guidance     |
+--------------------------+-----------------------+--------------------------------+
| Steering Linkage Slack / | Medium (Drift during  | Pre-loaded brass ball-joints + |
| Mechanical Wear          | straight runs)        | IMU heading error feedback loop|
+--------------------------+-----------------------+--------------------------------+
| Battery Voltage Drop     | High (Motor slowdown  | Low-noise buck regulator with  |
|                          | & camera reset)       | isolated logic power rail      |
+--------------------------+-----------------------+--------------------------------+
```

---

## 4. Iterative Development & Version History

```
  v1.0 (Initial Prototype)         v1.5 (Mid-Stage Development)           v2.0 (Final Finalist)
┌──────────────────────────┐     ┌───────────────────────────┐     ┌───────────────────────────┐
│ • Direct drive 1:10      │     │ • 1:16 Spur gearbox       │     │ • 1:24 2-Stage transmission│
│ • Acrylic laser chassis  │────►│ • 3D Printed PETG frame   │────►│ • Carbon fiber lower deck │
│ • Open-loop steering     │     │ • Single ToF sensor       │     │ • 4x ToF array + IMU      │
│ • Basic color threshold  │     │ • Closed-loop PID steering│     │ • Full FSM & Park Engine  │
└──────────────────────────┘     └───────────────────────────┘     └───────────────────────────┘
```
