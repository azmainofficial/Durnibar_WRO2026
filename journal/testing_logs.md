# Engineering Journal — Testing Logs & Metrics

> **WRO 2026 Future Engineers**  
> **Team Durnibar | Bangladesh**

---

## 1. Practice Run Performance Metrics

### Open Challenge Lap Testing (3-Lap Endurance Runs)

| Run ID | Corridor Width | Target Speed | Laps Completed | Total Time (s) | Avg Lap Time (s) | Result / Observations |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Run-01** | $1000\text{ mm}$ (Wide) | $1.5\text{ m/s}$ | 3 | $32.4\text{ s}$ | $10.8\text{ s}$ | Fast run; slight oscillation in corner 3 due to high $K_p$. |
| **Run-02** | $600\text{ mm}$ (Narrow) | $1.2\text{ m/s}$ | 3 | $38.1\text{ s}$ | $12.7\text{ s}$ | Smooth execution; narrow corridor maintained with $\pm 4\text{ cm}$ clearance. |
| **Run-03** | Mixed ($600/1000\text{ mm}$) | $1.2\text{ m/s}$ | 3 | $36.5\text{ s}$ | $12.17\text{ s}$ | Flawless turn transitions and autonomous finish section stop. |

---

## 2. Obstacle Avoidance & Parallel Parking Metrics

| Test Session | Traffic Signs Placed | Steering PID Setup | Pillar Avoidance % | Parking Alignment ($\Delta d$) | Parking Success Rate |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Session A** | 4 (2 Red, 2 Green) | $K_p=1.8, K_d=0.2$ | $80\%$ (1 bump) | $3.5\text{ cm}$ (Non-parallel) | $50\%$ |
| **Session B** | 6 (3 Red, 3 Green) | $K_p=1.4, K_d=0.4$ | $95\%$ | $1.8\text{ cm}$ (Parallel) | $85\%$ |
| **Session C** | 6 (3 Red, 3 Green) | $K_p=1.25, K_d=0.45$| **$100\%$** | **$0.8\text{ cm}$ (Parallel)** | **$95\%$** |

---

## 3. Computer Vision HSV Calibration Log

| Venue Lighting Condition | Ambient Lux | Red HSV Range | Green HSV Range | Detection Accuracy |
| :--- | :---: | :--- | :--- | :---: |
| **Lab Bench (Fluorescent)** | 450 Lux | `[0,120,70] - [10,255,255]` | `[35,80,70] - [85,255,255]` | $99.2\%$ |
| **Dim Indoor Room** | 180 Lux | `[0,90,50] - [10,255,255]` | `[30,60,50] - [85,255,255]` | $97.8\%$ |
| **Bright Competition Hall** | 850 Lux | `[0,140,80] - [10,255,255]` | `[38,90,80] - [85,255,255]` | $99.5\%$ |
