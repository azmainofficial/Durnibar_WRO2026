// =============================================================
// calibration.cpp  –  Calibration routines implementation.
// =============================================================
#include "calibration.h"
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "globals.h"

// ── Load from EEPROM (call once in setup) ─────────────────
void loadCalibration(CalibrationData &calData) {
  EEPROM.get(0, calData);
  if (!calData.isCalibrated) {
    calData.isCalibrated = false;
    calData.gyroXoff  = 0; calData.gyroYoff  = 0; calData.gyroZoff  = 0;
    calData.accelXoff = 0; calData.accelYoff = 0; calData.accelZoff = 0;
    calData.magXmin   = -3000; calData.magXmax = 3000;
    calData.magYmin   = -3000; calData.magYmax = 3000;
    calData.magZmin   = -3000; calData.magZmax = 3000;
  }
}

// ── Persist to EEPROM ─────────────────────────────────────
void saveCalibration(CalibrationData &calData) {
  calData.isCalibrated = true;
  EEPROM.put(0, calData);
  EEPROM.commit();
}

// ── Interactive calibration routine ───────────────────────
void runCalibration(CalibrationData &calData) {
  // ── Phase 1: gyro & accelerometer (keep board still) ────
  oled.clear();
  oled.println("=== CALIBRATION ===");
  oled.println("1. Keep STILL!");
  oled.println("   Calibrating MPU...");
  delay(2000);

  long sumGX = 0, sumGY = 0, sumGZ = 0;
  long sumAX = 0, sumAY = 0, sumAZ = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);
    if (Wire.available() >= 14) {
      sumAX += (Wire.read() << 8) | Wire.read();
      sumAY += (Wire.read() << 8) | Wire.read();
      sumAZ += (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();  // skip temperature
      sumGX += (Wire.read() << 8) | Wire.read();
      sumGY += (Wire.read() << 8) | Wire.read();
      sumGZ += (Wire.read() << 8) | Wire.read();
    }
    delay(4);
  }

  calData.gyroXoff  = sumGX / samples;
  calData.gyroYoff  = sumGY / samples;
  calData.gyroZoff  = sumGZ / samples;
  calData.accelXoff = sumAX / samples;
  calData.accelYoff = sumAY / samples;
  // 16384 LSB/g at ±2 g full-scale (MPU6050 default) → remove gravity from Z
  calData.accelZoff = (sumAZ / samples) - 16384;

  // ── Phase 2: magnetometer (rotate board in all directions) ─
  oled.clear();
  oled.println("=== CALIBRATION ===");
  oled.println("2. Rotate board in");
  oled.println("   all directions!");
  oled.println("   (10 Seconds)");
  delay(1000);

  int16_t mxMin =  32767, mxMax = -32768;
  int16_t myMin =  32767, myMax = -32768;
  int16_t mzMin =  32767, mzMax = -32768;

  unsigned long calStart = millis();
  while (millis() - calStart < 10000) {
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)QMC5883L_ADDR, (size_t)6, true);
    if (Wire.available() >= 6) {
      int16_t mx = Wire.read() | (Wire.read() << 8);
      int16_t my = Wire.read() | (Wire.read() << 8);
      int16_t mz = Wire.read() | (Wire.read() << 8);
      if (mx < mxMin) mxMin = mx;  if (mx > mxMax) mxMax = mx;
      if (my < myMin) myMin = my;  if (my > myMax) myMax = my;
      if (mz < mzMin) mzMin = mz;  if (mz > mzMax) mzMax = mz;
    }
    delay(20);
  }

  calData.magXmin = mxMin; calData.magXmax = mxMax;
  calData.magYmin = myMin; calData.magYmax = myMax;
  calData.magZmin = mzMin; calData.magZmax = mzMax;

  saveCalibration(calData);
  oled.clear();
  oled.println("CALIBRATION DONE!");
  oled.println("Saved to EEPROM.");
  delay(2000);
  oled.clear();
}
