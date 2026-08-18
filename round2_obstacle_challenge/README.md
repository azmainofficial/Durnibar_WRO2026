# Raspberry Pi 5 – Challenge 2 (Obstacle Challenge) Isolated Subsystem

This folder contains the isolated Raspberry Pi 5 scripts (vision, RPLiDAR C1, optimal path planning, and ESP32 telemetry) for **Challenge 2 (Obstacle Challenge)**.

---

## 🔌 Connection Details

| Field | Value |
|---|---|
| **Pi User** | `azmain` |
| **Pi IP** | `192.168.137.137` |
| **Remote Dir** | `~/pi_code/` |
| **SSH Password** | `123` |

---

## 📶 Network Modes — Works With OR Without WiFi

### Mode A — Connected to known WiFi / USB tethering
Dashboard at: **`http://192.168.137.137:5000`**

### Mode B — No network available (competition venue, field, etc.)
The Pi creates **its own WiFi hotspot** automatically:

| Field | Value |
|---|---|
| **Hotspot SSID** | `WRO-Robot` |
| **Password** | `wro12345` |
| **Dashboard URL** | `http://10.42.0.1:5000` |

Connect your phone or laptop to `WRO-Robot` and open the dashboard — no router needed.

> **One-time setup required.** Run `setup_hotspot.sh` once on the Pi (see below).

---

## 🚀 One-Time Setup on Pi (Run Once via SSH)

SSH into the Pi, then run these commands **once** to register the service and enable auto-start on every boot:

```bash
ssh azmain@192.168.137.137

# 1. Copy the service file to systemd
sudo cp ~/pi_code/wro_nav.service /etc/systemd/system/wro_nav.service

# 2. Reload systemd to pick up the new service
sudo systemctl daemon-reload

# 3. Enable the service to auto-start on every boot
sudo systemctl enable wro_nav.service

# 4. Set up the WiFi hotspot fallback (no-network mode)
bash ~/pi_code/setup_hotspot.sh

# 5. Start it immediately (no reboot needed)
sudo systemctl start wro_nav.service

# 6. Verify it is running
sudo systemctl status wro_nav.service
```

After this, every time you **power on** the Pi, the navigation system starts automatically within ~10 seconds.

---

## 🎮 Button Operation (After Boot)

The system starts in **IDLE** mode automatically. Buttons on the ESP32 control the challenge:

| Button | First Press | Press Again |
|---|---|---|
| **Button 1** | ▶ Start **Open Challenge** (3 laps) | ■ Stop / Go IDLE |
| **Button 2** | ▶ Start **Obstacle Challenge** (3 laps → Auto-Park) | ■ Stop / Go IDLE |

> After 3 laps in Open Challenge, the bot stops automatically in the designated area.
> After 3 laps in Obstacle Challenge, it automatically enters parallel parking mode.

---

## 📊 Live Web Dashboard

While the bot is running, open a browser on any device connected to the same network:

```
http://192.168.137.137:5000
```

The dashboard shows:
- **Live annotated camera feed** (16:9 640×360)
- **HSV binary mask preview**
- **RPLiDAR C1 radar canvas** with trajectory overlays
- **Real-time telemetry**: FPS, yaw, speed, wall distances, lap count
- **Planner controls**: Adjust weights, speed, FOV live

---

## 🔧 Service Management Commands

```bash
# View live logs from the service
journalctl -u wro_nav -f

# Restart the service (e.g. after deploying new code)
sudo systemctl restart wro_nav.service

# Stop the service
sudo systemctl stop wro_nav.service

# Disable auto-start (if needed)
sudo systemctl disable wro_nav.service
```

---

## 📦 Deploying Code Updates from PC

Run this from your Windows PC to push all updated files to the Pi:

```bash
python pi_code/deploy_to_pi.py
```

After deploying, restart the service on the Pi:
```bash
ssh azmain@192.168.137.137 "sudo systemctl restart wro_nav.service"
```

---

## 🧪 Sensor Diagnostics

Before running a competition, verify all hardware is working:

```bash
ssh azmain@192.168.137.137 "cd ~/pi_code && python3 test_sensors.py"
```

Expected output:
```
- Camera (Fifine K420) : PASS [OK]
- RPLiDAR C1 Scanner   : PASS [OK]
```
