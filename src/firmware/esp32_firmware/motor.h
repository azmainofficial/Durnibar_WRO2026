// =============================================================
// motor.h  –  DC motor driver (TB6612FNG) via ESP32 LEDC PWM.
//
// Provides:
//   initMotor()        – configure GPIO & attach 20 kHz PWM
//   setMotorSpeed(int) – drive motor -255 (full rev) to +255 (full fwd),
//                        0 = active brake (both AIN HIGH on TB6612)
// =============================================================
#pragma once

// ── Initialise motor GPIO & PWM ───────────────────────────
void initMotor();

// ── Set motor speed ───────────────────────────────────────
void setMotorSpeed(int speed);

