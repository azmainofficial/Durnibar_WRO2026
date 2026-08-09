# Criterion 1: Mobility and Mechanical Design

> **WRO 2026 Future Engineers — Technical Report**  
> **Team Durnibar | Bangladesh**

---

## 1. Overview & Mechanical Requirements

According to section 11 of the **WRO 2026 Future Engineers General Rules**, the robotic vehicle must comply with strict physical constraints:
- **Maximum Dimensions**: $300 \text{ mm (Length)} \times 200 \text{ mm (Width)} \times 300 \text{ mm (Height)}$.
- **Maximum Mass**: $1.5 \text{ kg}$.
- **Drive Kinematics**: 4-wheeled vehicle featuring **one single driving axle** (front, rear, or 4WD) and **one steering actuator** (Rule 11.3).
- **Prohibited**: Differential drive bases (skid steering), omni wheels, ball casters, and electronic differentials using independent side motors (Rule 11.4 & 11.5).

---

## 2. Steering Kinematics: Ackermann Steering Mechanism

To achieve smooth cornering without tire scrub on the narrow $600\text{ mm} - 1000\text{ mm}$ track corridors, **Team Durnibar** implemented an **Ackermann Steering Mechanism**.

```
         ┌────────────────────────┐
         │     Front Axle         │
  Wheel  │   o─────────o          │ Wheel
 ┌─────┐ │  /           \         │ ┌─────┐
 │     │─┼─o  Steering   o───────┼─│     │
 └─────┘ │  \   Tie-Rod /         │ └─────┘
         │   └───o───o─┘          │
         │       │   │            │
         │     Servo Lever        │
         └────────────────────────┘
```

### Ackermann Geometry Calculations
For a car with wheelbase $L = 175\text{ mm}$ and track width $W = 145\text{ mm}$, the inner wheel angle $\delta_i$ and outer wheel angle $\delta_o$ during a turn of radius $R$ must satisfy:

$$\cot(\delta_o) - \cot(\delta_i) = \frac{W}{L}$$

$$\delta_o = \arctan\left(\frac{L}{\frac{L}{\tan(\delta_i)} + W}\right)$$

This mechanism ensures all four wheels trace concentric circles around a common Instantaneous Center of Rotation (ICR), preventing sideways drag and power loss during high-speed cornering.

---

## 3. Drive System & Gear Box Design

The drive system utilizes a single high-torque brushless DC motor connected to the rear axle through a custom **2-stage spur gear transmission** designed in Fusion 360 (`models/cad_source/Gear Box v2.0.f3d`).

### Gear Ratio & Speed/Torque Reasoning
- **Motor Rated Speed**: $8,500 \text{ RPM}$ at $11.1 \text{ V}$.
- **Target Robot Top Speed**: $1.2 \text{ m/s}$ (optimal for vision processing latency at $30 \text{ FPS}$).
- **Wheel Diameter ($D$)**: $65 \text{ mm}$ ($\text{Circumference } C = \pi \times 65 \text{ mm} = 0.2042 \text{ m}$).

Target Wheel RPM ($N_{wheel}$):
$$N_{wheel} = \frac{v}{C} \times 60 = \frac{1.2 \text{ m/s}}{0.2042 \text{ m}} \times 60 \approx 352.6 \text{ RPM}$$

Required Total Gear Reduction Ratio ($i_{total}$):
$$i_{total} = \frac{N_{motor}}{N_{wheel}} = \frac{8500}{352.6} \approx 24.1:1$$

We selected a **24:1 gear ratio** ($1:4$ first stage, $1:6$ second stage), providing:
- **Output Speed**: $1.16 \text{ m/s}$.
- **Stall Torque Multiplier**: $24\times$, delivering ample torque for instant acceleration from rest and reliable parallel parking maneuvers.

---

## 4. Chassis Layout & Material Selection

The chassis (`models/cad_source/Durnibar 2.0.f3z`) uses a dual-deck architecture:
1. **Lower Deck**: 3mm Carbon Fiber / acrylic sheet housing the rear drive assembly, steering servo, battery compartment, and low-level power electronics.
2. **Upper Deck**: 3D-printed PETG plates holding the Raspberry Pi / SBC main compute unit, camera mount, ToF sensors, and IMU.

```
+-------------------------------------------------------------+
|                        UPPER DECK                           |
|   [Camera Mount]     [Raspberry Pi / SBC]     [IMU]         |
+-------------------------------------------------------------+
|                        LOWER DECK                           |
|  [Steering Servo]  [LiPo 3S Battery]  [Motor & Gearbox]     |
+-------------------------------------------------------------+
```

---

## 5. Iterative Design & Testing History

| Iteration | Drive Setup | Steering | Key Findings & Improvements |
| :--- | :--- | :--- | :--- |
| **v1.0** | 1:10 Gear Ratio Direct Drive | 3D Printed Steering Links | High top speed ($2.1 \text{ m/s}$), but insufficient torque for low-speed parking; steering links had excess backlash. |
| **v1.5** | 1:16 Spur Gearbox | Metal Ball-Joint Tie-Rods | Improved steering precision ($\pm 0.5^\circ$ repeatability); reduced motor heating during continuous lap testing. |
| **v2.0 (Current)**| 2-Stage 1:24 Sealed Gearbox | Integrated Servo Saver + Ackermann | Sealed transmission prevents dust ingress; integrated servo saver protects gear teeth during wall bumps. |
