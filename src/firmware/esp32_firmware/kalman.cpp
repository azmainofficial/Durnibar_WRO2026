// =============================================================
// kalman.cpp  –  Discrete Kalman filter implementation.
// =============================================================
#include "kalman.h"

Kalman::Kalman() {
  Q_angle   = 0.001f;   // process noise: angle
  Q_bias    = 0.003f;   // process noise: gyro bias
  R_measure = 0.03f;    // measurement noise
  angle     = 0.0f;
  bias      = 0.0f;
  P[0][0]   = 0.0f; P[0][1] = 0.0f;
  P[1][0]   = 0.0f; P[1][1] = 0.0f;
}

// Call every sample period.
// newAngle  – angle from accelerometer [degrees]
// newRate   – angular rate from gyroscope [degrees/s]
// dt        – elapsed time since last call [seconds]
// returns   – filtered angle [degrees]
float Kalman::getAngle(float newAngle, float newRate, float dt) {
  // Predict
  rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
  float S    = P[0][0] + R_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  float y    = newAngle - angle;

  angle += K[0] * y;
  bias  += K[1] * y;

  float P00_tmp = P[0][0];
  float P01_tmp = P[0][1];
  P[0][0] -= K[0] * P00_tmp;
  P[0][1] -= K[0] * P01_tmp;
  P[1][0] -= K[1] * P00_tmp;
  P[1][1] -= K[1] * P01_tmp;

  return angle;
}
