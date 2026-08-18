---
name: sulg-slam-robot
description: Comprehensive ROS 2 Jazzy SLAM, RPLiDAR C1 driver management, Ackermann hardware bridge, and Ceres-solver workflow guide for sulg_robot.
---

# ROS 2 SLAM & Hardware Management Skill (`sulg_robot`)

This skill defines the authoritative operations, hardware protocols, driver configurations, and ROS 2 Jazzy workflows for the `sulg_robot` Ackermann-steering platform.

---

## 1. Hardware Architecture & Connectivity

* **Host Computer (Raspberry Pi 5)**:
  * Address: `azmain@192.168.1.101` (SSH password: `123`)
  * Workspace Path: `~/ws_lidar` (`src/sulg_robot`, `src/sllidar_ros2`)
* **RPLiDAR C1 Scanner**:
  * Port: `/dev/ttyUSB0` (CP2102 USB Bridge)
  * Persistent Symlink: `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4d6c14f3473ed11b23e6aeefdf7b791-if00-port0`
  * Baud Rate: `460800` baud
  * Operating Scan Mode: Auto (`scan_mode: ''` -> negotiates `DenseBoost` 40m range @ 10 Hz)
  * Physical Mount Orientation: Flipped/Inverted (`'inverted': 'true'`, `laser_joint rpy: 3.14159 0 0`)
* **ESP32 Motor & Steering Controller**:
  * Port: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_56BA018173-if00` (or `/dev/ttyACM0`)
  * Baud Rate: `115200` baud
  * Protocol: Serial commands (`D <speed_pwm> <steer_pwm>`, `R` reset, `S` estop)

---

## 2. Critical Driver & System Guidelines

### A. RPLiDAR C1 Driver Rules (`sllidar_ros2`)
1. **Auto Scan Mode Negotiation**: Always set `'scan_mode': ''` in launch files. Never hardcode `'Standard'` or `'Express'` mode; explicit express modes invoke `startScanExpress` in the C++ SDK which causes `0x80008002` timeouts over 460.8k baud.
2. **Loop Error Handling**: In `sllidar_node.cpp`, frame-grab failures must execute `usleep(10000)` without calling `drv->stop()`. Calling `drv->stop()` during active scan cycles power to the motor and causes pulsating spin-stop behavior.
3. **Initial Connect Retries**: `sllidar_node.cpp` must maintain a 5-attempt reconnect loop with 500ms delays to handle transient serial open timing.

### B. Hardware Bridge One-Time Motor Latch (`sulg_hardware_bridge`)
* In `hardware_bridge.py`, the `/start_motor` service client MUST cancel its ROS timer (`self.motor_timer.cancel()`) after triggering once at bringup. Periodically calling `/start_motor` resets PWM and interrupts laser scanning.

### C. USB Power & Autosuspend
* Disable USB autosuspend on Raspberry Pi to prevent CP2102 DTR drop errors (`failed set request 0x12 status: -110`):
  ```bash
  echo "on" | sudo tee /sys/bus/usb/devices/*/power/control
  echo -1 | sudo tee /sys/bus/usb/devices/*/power/autosuspend_delay_ms
  ```

### D. ROS 2 Jazzy `slam_toolbox` Solver Plugin
* `slam_toolbox` in ROS 2 Jazzy requires `solver_plugin: solver_plugins::CeresSolver` in `mapper_params_online_async.yaml`. Legacy `CsparseSolver` is incompatible.

---

## 3. Remote Execution & Launch Workflows

### Clean Process Shutdown
Always stop active node graphs before re-launching:
```bash
echo 123 | sudo -S killall -9 python3 ros2 launch_ros sllidar_node async_slam_toolbox_node robot_state_publisher sulg_hardware_bridge sulg_vision_detector sulg_web_bridge 2>/dev/null || true
```

### Remote Workspace Build
```bash
bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/ws_lidar && colcon build --packages-select sllidar_ros2 sulg_robot'
```

### Launch SLAM (Pi 5)
```bash
sshpass -p '123' ssh -o StrictHostKeyChecking=no azmain@192.168.1.101 "bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ws_lidar/install/setup.bash && ros2 launch sulg_robot sulg_slam.launch.py'"
```

### Launch RViz2 (Local PC)
```bash
bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true; source '/media/azmainofficial/New Volume/Sulg/ros2_ws/install/setup.bash' && ros2 launch sulg_robot sulg_rviz.launch.py"
```
