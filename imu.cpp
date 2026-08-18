// =============================================================
// imu.cpp  –  MPU6050 + QMC5883L driver and sensor fusion implementation.
// =============================================================
#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "globals.h"

// ── Initialise MPU6050 ────────────────────────────────────
// Clears SLEEP bit and configures full-scale ranges & low-pass filter.
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);  // wake up (clear SLEEP bit)
  if (Wire.endTransmission(true) != 0) return false;

  // Gyro full-scale range = +/-250 deg/s (131 LSB per deg/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // FS_SEL = 0
  if (Wire.endTransmission(true) != 0) return false;

  // Accel full-scale range = +/-2g (16384 LSB per g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // AFS_SEL = 0
  if (Wire.endTransmission(true) != 0) return false;

  // Digital Low Pass Filter (DLPF) = 42 Hz for smooth readings
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG
  Wire.write(0x03);  // DLPF_CFG = 3
  return (Wire.endTransmission(true) == 0);
}

// ── Initialise QMC5883L magnetometer ─────────────────────
// Register 0x09: ODR=200 Hz, full-scale ±8 G, OSR=512, mode=Continuous.
bool initMagnetometer() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);  // OSR=512, FS=8G, ODR=200Hz, mode=Continuous
  return (Wire.endTransmission(true) == 0);
}

// ── Read sensors & update Kalman + heading ────────────────
// Must be called frequently (every loop iteration) for accurate dt.
void updateIMU() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;

  // ── MPU6050: read accel (14 bytes includes temp) ─────
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);
  if (Wire.available() >= 14) {
    int16_t rawAX = ((Wire.read() << 8) | Wire.read()) - calData.accelXoff;
    int16_t rawAY = ((Wire.read() << 8) | Wire.read()) - calData.accelYoff;
    int16_t rawAZ = ((Wire.read() << 8) | Wire.read()) - calData.accelZoff;
    Wire.read(); Wire.read();  // skip temperature bytes
    int16_t rawGX = ((Wire.read() << 8) | Wire.read()) - calData.gyroXoff;
    int16_t rawGY = ((Wire.read() << 8) | Wire.read()) - calData.gyroYoff;
    int16_t rawGZ = ((Wire.read() << 8) | Wire.read()) - calData.gyroZoff;

    // Accelerometer angles
    float accRoll  = atan2(rawAY, rawAZ) * RAD_TO_DEG;
    float accPitch = atan2(-rawAX, sqrt((float)rawAY * rawAY +
                                        (float)rawAZ * rawAZ)) * RAD_TO_DEG;

    // Gyro rates (131 LSB per °/s at ±250 °/s full-scale)
    float gyroXrate = rawGX / 131.0f;
    float gyroYrate = rawGY / 131.0f;
    float gyroZrate = rawGZ / 131.0f;

#if defined(INVERT_GYRO_Z) && (INVERT_GYRO_Z == true)
    gyroZrate = -gyroZrate;
#endif

    // Fuse roll & pitch with Kalman
    kalRoll  = kalmanRoll.getAngle(accRoll,  gyroXrate, dt);
    kalPitch = kalmanPitch.getAngle(accPitch, gyroYrate, dt);

    // Integrate Gyro Z rate for Yaw (heading)
    imuYaw += gyroZrate * dt;
    if (imuYaw < 0.0f)   imuYaw += 360.0f;
    if (imuYaw >= 360.0f) imuYaw -= 360.0f;
  }

  // ── QMC5883L: read magnetometer (6 bytes, LSB first) ─
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)QMC5883L_ADDR, (size_t)6, true);
  if (Wire.available() >= 6) {
    int16_t mx = Wire.read() | (Wire.read() << 8);
    int16_t my = Wire.read() | (Wire.read() << 8);
    int16_t mz = Wire.read() | (Wire.read() << 8);

    // Hard-iron correction (offset = midpoint of min/max range)
    float magX = mx - (calData.magXmin + calData.magXmax) / 2.0f;
    float magY = my - (calData.magYmin + calData.magYmax) / 2.0f;
    float magZ = mz - (calData.magZmin + calData.magZmax) / 2.0f;

    // Tilt compensation: rotate mag vector into horizontal plane
    float rollRad  = kalRoll  * DEG_TO_RAD;
    float pitchRad = kalPitch * DEG_TO_RAD;
    float magXh = magX * cos(pitchRad) + magZ * sin(pitchRad);
    float magYh = magX * sin(rollRad) * sin(pitchRad)
                + magY * cos(rollRad)
                - magZ * sin(rollRad) * cos(pitchRad);

    magHeading = atan2(magYh, magXh) * RAD_TO_DEG;
    if (magHeading < 0) magHeading += 360.0f;

    // Relative startup zeroing: first reading sets the reference origin (0.0 degrees)
    static float initialMagHeading = -1.0f;
    if (initialMagHeading < 0.0f) {
      initialMagHeading = magHeading;
    }

    // Apply magnetometer drift correction ONLY when the motor is stopped (motorSpeed == 0)
    // When the motor is running, DC currents create strong magnetic EMI that distorts turns.
    if (motorSpeed == 0) {
      float relMagHeading = magHeading - initialMagHeading;
      if (relMagHeading < 0.0f)   relMagHeading += 360.0f;
      if (relMagHeading >= 360.0f) relMagHeading -= 360.0f;

      float headingDiff = relMagHeading - imuYaw;
      while (headingDiff < -180.0f) headingDiff += 360.0f;
      while (headingDiff > 180.0f)  headingDiff -= 360.0f;

      // 2% correction from magnetometer to eliminate stationary gyro Z drift
      imuYaw += headingDiff * 0.02f;
      if (imuYaw < 0.0f)   imuYaw += 360.0f;
      if (imuYaw >= 360.0f) imuYaw -= 360.0f;
    }
  }
}



