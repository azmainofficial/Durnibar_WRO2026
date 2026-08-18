// =============================================================
// kalman.h  –  Discrete Kalman filter for angle estimation.
// Self-contained; no dependencies on other project headers.
//
// Usage:
//   Kalman kf;
//   float angle = kf.getAngle(accelAngle, gyroRate, dt_seconds);
// =============================================================
#pragma once

class Kalman {
public:
  Kalman();

  // Call every sample period.
  // newAngle  – angle from accelerometer [degrees]
  // newRate   – angular rate from gyroscope [degrees/s]
  // dt        – elapsed time since last call [seconds]
  // returns   – filtered angle [degrees]
  float getAngle(float newAngle, float newRate, float dt);

private:
  float Q_angle, Q_bias, R_measure;
  float angle, bias, rate;
  float P[2][2];
};

