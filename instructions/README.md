
---

```markdown
# ROS2 Jazzy on Raspberry Pi 5 (Ubuntu Server 24.04)

This guide documents the complete installation of **ROS2 Jazzy** on a **Raspberry Pi 5 (8GB)** running **Ubuntu Server 24.04 LTS (64-bit)**. It includes fixing network issues, installing ROS2, testing with talker/listener, and optional fan control.

---

## 🖥️ System Overview

- **Hardware:** Raspberry Pi 5, 8GB RAM, Active Cooler
- **OS:** Ubuntu Server 24.04 LTS (Noble) – headless, minimal for robotics
- **ROS2 Distribution:** Jazzy Jalisco (LTS, supported until 2029)
- **Use Case:** LiDAR, computer vision, SLAM (resource‑optimised)

---

## 1. Operating System Setup

Flash Ubuntu Server 24.04 (64-bit) using **Raspberry Pi Imager**:

- Choose OS: `Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)`
- Pre‑configure: hostname (`pi-robot`), username (`azmain`), password, Wi‑Fi, **enable SSH**
- Boot the Pi and connect:
  ```bash
  ssh azmain@192.168.1.114   # use your Pi’s actual IP
  ```

---

## 2. Fixing DNS / `raw.githubusercontent.com` Resolution

**Problem:** `curl` failed with `Could not resolve host: raw.githubusercontent.com`

**Solution:** Manually add the IP address to `/etc/hosts`.

1. Find a working IP (from another machine):
   ```bash
   ping raw.githubusercontent.com
   ```
   Example IP: `185.199.108.133`

2. Edit the hosts file on the Pi:
   ```bash
   sudo nano /etc/hosts
   ```
   Add this line at the end:
   ```
   185.199.108.133 raw.githubusercontent.com
   ```
   Save (`Ctrl+O`, `Enter`, `Ctrl+X`).

3. (Optional) Prevent cloud-init from overwriting the change:
   ```bash
   sudo cloud-init devel add-config -s 'manage_etc_hosts: False'
   ```

Now `curl` commands to GitHub work correctly.

---

## 3. Installing ROS2 Jazzy

### 3.1 Set locale
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.2 Add ROS2 repository
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3.3 Install ROS2 base (headless, minimal)
```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-base
```

### 3.4 Install development tools
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool git
sudo rosdep init
rosdep update
```

### 3.5 Source ROS2 environment automatically
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Installing Demo Nodes (for testing)

The `ros-base` installation does not include the demo talker/listener. Install them:

```bash
sudo apt update
sudo apt install -y ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
```

---

## 5. Verifying the Installation

Open **two separate SSH terminals** to your Pi.

**Terminal 1 (talker):**
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (listener):**
```bash
ros2 run demo_nodes_py listener
```

✅ **Success:** Listener prints `I heard: [Hello World: ...]` repeatedly.

---

## 6. Optional: Raspberry Pi 5 Active Cooler Control

The fan is temperature‑controlled and may not spin until CPU >50°C. For heavy SLAM/vision, you may want to adjust the fan curve.

### 6.1 Install `pinctrl` (to manually test the fan)
```bash
sudo apt install -y cmake build-essential
git clone https://github.com/raspberrypi/utils.git
cd utils/pinctrl
cmake .
make
sudo make install
```

### 6.2 Manual fan test
```bash
sudo pinctrl FAN_PWM a0          # disable PWM mode
sudo pinctrl FAN_PWM op dh       # set pin high → fan spins at full speed
sudo pinctrl FAN_PWM op dl       # set pin low → fan off
sudo pinctrl FAN_PWM a0          # restore PWM mode for automatic control
```

### 6.3 Custom automatic fan curve (aggressive cooling)
Edit `/boot/firmware/config.txt`:
```bash
sudo nano /boot/firmware/config.txt
```

Add these lines at the end:
```
# Custom fan curve for robotics workloads
dtparam=fan_temp0=45000   # start at 45°C
dtparam=fan_temp0_speed=50
dtparam=fan_temp1=55000
dtparam=fan_temp1_speed=100
dtparam=fan_temp2=65000
dtparam=fan_temp2_speed=180
dtparam=fan_temp3=75000
dtparam=fan_temp3_speed=255
```

Reboot: `sudo reboot`

Monitor temperature:
```bash
watch -n 1 vcgencmd measure_temp
```

---

## 7. Next Steps for Robotics

With ROS2 running, install the required packages for your robot:

| Package | Command |
|---------|---------|
| SLAM Toolbox | `sudo apt install ros-jazzy-slam-toolbox` |
| Navigation2 | `sudo apt install ros-jazzy-navigation2` |
| USB Camera | `sudo apt install ros-jazzy-usb-cam` |
| RPLIDAR driver | `sudo apt install ros-jazzy-sllidar` (or build from source) |

---

## 8. Troubleshooting

| Issue | Solution |
|-------|----------|
| `ros2: command not found` | Run `source /opt/ros/jazzy/setup.bash` or reopen terminal |
| `Package 'demo_nodes_cpp' not found` | Install `ros-jazzy-demo-nodes-cpp` as shown above |
| `curl: (6) Could not resolve host` | Follow Section 2 to add GitHub IP to `/etc/hosts` |
| Fan never spins | Check physical connection; try manual test (Section 6.2) |

---

## 9. Key Commands Summary

```bash
# Update system and install ROS2
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-ros-base

# Source environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Test ROS2
ros2 run demo_nodes_cpp talker     # terminal 1
ros2 run demo_nodes_py listener    # terminal 2

# Monitor temperature
watch -n 1 vcgencmd measure_temp
```

---

## 📚 References

- [ROS2 Jazzy Installation Docs](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
- [Raspberry Pi 5 Fan Control](https://www.raspberrypi.com/documentation/computers/raspberry-pi-5.html#fan-control)
- [Ubuntu Server for Raspberry Pi](https://ubuntu.com/download/raspberry-pi)

---

**Your Raspberry Pi 5 is now ready for advanced robotics with ROS2.**  
Happy building! 🚀
```

---

You can save this file on your Pi (e.g., `~/README.md`) and even commit it to a Git repository for your project. Let me know if you'd like to adjust any section!
