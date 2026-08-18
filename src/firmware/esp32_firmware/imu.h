// =============================================================
// imu.h  –  MPU6050 + QMC5883L driver and sensor fusion.
//
// Provides:
//   initMPU6050()     – wake the gyro/accel, returns false on error
//   initMagnetometer()– configure QMC5883L, returns false on error
//   updateIMU()       – read both sensors, run Kalman filters,
//                       compute tilt-compensated magnetic heading
// =============================================================
#pragma once

#include <Arduino.h>

// ── Initialise MPU6050 ────────────────────────────────────
bool initMPU6050();

// ── Initialise QMC5883L magnetometer ─────────────────────
bool initMagnetometer();

// ── Read sensors & update Kalman + heading ────────────────
void updateIMU();

