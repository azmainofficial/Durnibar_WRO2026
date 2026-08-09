# Robot Source Code Directory

> **WRO 2026 Future Engineers — Team Durnibar**

This folder contains the complete, well-commented source code powering **Durnibar 2.0**.

---

## 📁 Source Modules

```
src/
├── main_node/       # High-level ROS 2 / Python decision node & Finite State Machine
├── vision/          # OpenCV image processing & color pillar detection pipeline
├── navigation/      # PID wall-following, Ackermann steering calculation & parking engine
└── firmware/        # Low-level ESP32 C++/Arduino firmware (PWM drivers, ToF sensors, serial bridge)
```

---

## 🛠 Compilation & Deployment

For complete build and flashing instructions, refer to **[docs/05_reproducibility_guide.md](../docs/05_reproducibility_guide.md)**.
