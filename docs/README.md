# Team Durnibar — Technical Documentation Directory

Welcome to the technical documentation repository for **Team Durnibar** participating in the **WRO 2026 Future Engineers (Self-Driving Cars)** category. 

This folder contains comprehensive engineering documentation structured directly around the **WRO 2026 Official Rules and Appendix C Judging Criteria**.

---

## 📑 Documentation Structure

Our documentation is split into five core technical reports corresponding to the judging rubrics:

| File | Judging Criterion | Focus & Key Contents |
| :--- | :--- | :--- |
| **[01_mobility_and_mechanics.md](./01_mobility_and_mechanics.md)** | **Criterion 1: Mobility & Mechanical Design** | Ackermann steering geometry, single rear-axle differential drive, gear ratio & torque calculations, CAD iterations, chassis rigidity. |
| **[02_power_and_sensors.md](./02_power_and_sensors.md)** | **Criterion 2: Power & Sensor Architecture** | Power budget & distribution, LiPo battery regulation, ToF/LiDAR/Camera placement geometry, pinout map, sensor calibration. |
| **[03_software_and_obstacle.md](./03_software_and_obstacle.md)** | **Criterion 3: Software Architecture & Obstacle Strategy** | Modular software pipeline, Finite State Machine (FSM), OpenCV pillar recognition (Red=Right, Green=Left), PID steering control, Parallel Parking algorithm. |
| **[04_systems_thinking_decisions.md](./04_systems_thinking_decisions.md)** | **Criterion 4: Systems Thinking & Engineering Decisions** | Architectural tradeoffs, physical & computational constraints, failure mode matrix & mitigation, version history (v1.0 -> v2.0). |
| **[05_reproducibility_guide.md](./05_reproducibility_guide.md)** | **Criterion 5: Reproducibility & Build Guide** | Complete Bill of Materials (BOM), step-by-step assembly guide, software compilation & firmware flashing instructions. |

---

## 🎯 Scoring & Evaluation Quick Reference

According to **Appendix C of the WRO 2026 General Rules**, the documentation is evaluated out of **30 points** across these 5 criteria:

- **6 Points per Criterion**: Requires advanced engineering, quantitative trade-off analysis, explicit formulas/calculations, failure handling, testing metrics, and complete reproducibility.
- **Rules Compliance Checklist**:
  - Maximum dimensions: $300 \text{ mm (L)} \times 200 \text{ mm (W)} \times 300 \text{ mm (H)}$.
  - Maximum weight: $1.5 \text{ kg}$.
  - Single drive axle with physical differential / connected wheels (Rule 11.3 & 11.13). Independent motors per side / differential drive bases are strictly prohibited.
  - Exactly one main power switch and one start button (Rule 9.10 & 9.11).
