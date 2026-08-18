// =============================================================
// calibration.h  –  CalibrationData struct, EEPROM load/save,
//                   and the interactive calibration routine.
// =============================================================
#pragma once

#include <Arduino.h>

// ── Persistent calibration data structure ─────────────────
// Stored at EEPROM address 0.  isCalibrated acts as a validity flag.
struct CalibrationData {
  bool    isCalibrated;
  int16_t gyroXoff,  gyroYoff,  gyroZoff;
  int16_t accelXoff, accelYoff, accelZoff;
  int16_t magXmin,   magXmax;
  int16_t magYmin,   magYmax;
  int16_t magZmin,   magZmax;
};

// ── Load from EEPROM (call once in setup) ─────────────────
void loadCalibration(CalibrationData &calData);

// ── Persist to EEPROM ─────────────────────────────────────
void saveCalibration(CalibrationData &calData);

// ── Interactive calibration routine ───────────────────────
void runCalibration(CalibrationData &calData);

