# Criterion 3: Software Architecture and Obstacle Strategy

> **WRO 2026 Future Engineers — Technical Report**  
> **Team Durnibar | Bangladesh**

---

## 1. Software Modular Architecture

The control software for **Durnibar 2.0** uses a modular architecture running on ROS 2 / Python on the high-level Single Board Computer (SBC) linked via high-speed serial UART to the low-level MCU.

```
┌────────────────────────────────────────────────────────────────────────┐
│                        HIGH-LEVEL COMPUTE (SBC)                        │
│                                                                        │
│   ┌─────────────────────┐    ┌──────────────────────────────────┐      │
│   │ Vision Node         │    │ Navigation & Path Planner        │      │
│   │ • HSV Color Segmentation │───►│ • PID Wall Distance Controller  │      │
│   │ • Traffic Sign ID   │    │ • Ackermann Steering Calculator  │      │
│   └─────────────────────┘    └──────────────────────────────────┘      │
│                                               │                        │
│                                               ▼                        │
│                              ┌──────────────────────────────────┐      │
│                              │ Finite State Machine (FSM Engine)│      │
│                              └──────────────────────────────────┘      │
└───────────────────────────────────────────────┬────────────────────────┘
                                                │ Serial UART (115200 Baud)
┌───────────────────────────────────────────────▼────────────────────────┐
│                      LOW-LEVEL CONTROLLER (MCU)                        │
│   • PWM Motor Driver  • Servo Steering Control  • Sensor Polling       │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 2. High-Level Finite State Machine (FSM)

The vehicle's state machine governs behavior throughout both the **Open Challenge** and **Obstacle Challenge**.

```
    ┌───────────────┐
    │  BOOT_WAIT    │ ◄── Power On (Waiting for Start Button - Rule 9.11)
    └───────┬───────┘
            │ Button Pressed
            ▼
    ┌───────────────┐
    │  START_LEAVE  │ ◄── Exit Start Section & Init Lap Counter
    └───────┬───────┘
            │ Exit Confirmed
            ▼
    ┌───────────────┐      Pillar Detected       ┌──────────────────┐
    │  LANE_FOLLOW  ├───────────────────────────►│ OBSTACLE_EVADE   │
    └───────▲───────┘                            └────────┬─────────┘
            │                                             │ Evaded
            └─────────────────────────────────────────────┘
            │ Laps == 3 Completed
            ▼
    ┌───────────────┐     Obstacle Round?       ┌──────────────────┐
    │ LAP3_COMPLETE ├───────────────────────────►│  PARK_SEARCH     │
    └───────┬───────┘                           └────────┬─────────┘
            │ Open Round                                  │ Lot Found
            ▼                                             ▼
    ┌───────────────┐                           ┌──────────────────┐
    │ AUTONOMOUS_STOP│                           │ PARALLEL_PARK    │
    └───────────────┘                           └──────────────────┘
```

---

## 3. Traffic Sign Recognition & Avoidance Logic (Obstacle Challenge)

Per **WRO 2026 Rules Section 9.19**:
- **Red Traffic Sign (Pillar)**: Robot must pass on the **RIGHT** side.
- **Green Traffic Sign (Pillar)**: Robot must pass on the **LEFT** side.

### Color Segmentation & Spatial Bounding Box Filter
Images captured from the wide-angle camera are converted to **HSV color space** to build robust masks independent of venue light intensity variations:

```python
# HSV Threshold Ranges
RED_LOWER1 = np.array([0, 120, 70]);   RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 70]); RED_UPPER2 = np.array([180, 255, 255])
GREEN_LOWER = np.array([35, 80, 70]);  GREEN_UPPER = np.array([85, 255, 255])
```

```
           TRAFFIC SIGN PASSING STRATEGY

     Red Pillar (Keep Right)        Green Pillar (Keep Left)
        ┌───┐                          ┌───┐
        │   │                          │   │
        └───┘                          └───┘
          ▲                              ▲
          │  \                           │  /
          │   \ Vehicle Steering         │   / Vehicle Steering
          │    \ Shift Right             │    \ Shift Left
        ┌───┐  └───►                   ┌───┐  └───►
        │CAR│                          │CAR│
        └───┘                          └───┘
```

---

## 4. Steering Control: Proportional-Integral-Derivative (PID)

Steering angle $\delta$ is continuously computed from lateral displacement error $e(t)$ relative to target lane center line:

$$e(t) = d_{\text{target}} - d_{\text{measured}}$$

$$\delta(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

### Tuned PID Coefficients
- **$K_p = 1.25$**: Responsive steering corrections for sharp cornering.
- **$K_i = 0.02$**: Eliminates steady-state offset during long straightaways.
- **$K_d = 0.45$**: Dampens steering oscillation near corridor boundaries.

---

## 5. Autonomous Parallel Parking Algorithm

After completing **3 full laps** in the Obstacle Challenge, the vehicle initiates the parallel parking maneuver into the $20\text{ cm}$ wide parking lot (Rules 8 & Appendix A.6):

1. **Lot Scanning**: Side ToF sensors measure depth step change ($>20\text{ cm}$) bounded by magenta wood markers.
2. **Reverse Entry (Phase 1)**: Robot turns steering to max angle ($\delta = +28^\circ$) while reversing until rear corner clears inner boundary.
3. **Counter Steering (Phase 2)**: Robot reverses with opposite steering ($\delta = -28^\circ$) until aligned parallel to outer wall ($\Delta d < 2\text{ cm}$).
4. **Final Stop**: Full autonomous stop; projection fully inside lot without touching boundary blocks.
