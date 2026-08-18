# ROS 2 & ESP32 Source Code Directory

> **WRO 2026 Future Engineers — Team Durnibar**

This folder contains the complete, modular ROS 2 node architecture and low-level ESP32 C++ firmware powering **Durnibar 2.0**.

---

## 📁 Source Package Architecture

```
src/
├── durnibar_bringup/    # ROS 2 bringup launch files (RPLIDAR C1 + Fifine K420 camera + FSM)
├── durnibar_nav/        # RPLIDAR C1 360° LaserScan navigation & PID steering node
├── durnibar_vision/     # Fifine K420 108° camera OpenCV color pillar detection node
├── durnibar_fsm/        # High-level WRO 2026 Finite State Machine node
└── firmware/            # ESP32 C++ firmware & Arduino IDE / PlatformIO code
```

---

## 🚀 Building the ROS 2 Packages (Raspberry Pi 5)

```bash
# 1. Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# 2. Build workspace using colcon
cd ~/ros2_ws
colcon build --symlink-install

# 3. Source installation
source install/setup.bash

# 4. Launch full robot system (RPLIDAR C1 + Camera + Navigation + FSM)
ros2 launch durnibar_bringup robot_launch.py
```
