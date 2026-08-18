// =============================================================
// globals.h  –  All shared mutable state in one place.
// Every other module includes this file so they can read/write
// the same variables without extern gymnastics.
// =============================================================
#pragma once

#include <ESP32Servo.h>
#include "SSD1306AsciiWire.h"
#include "kalman.h"          // Kalman class must be visible before the instances below
#include "calibration.h"     // CalibrationData definition

// ── Hardware objects ──────────────────────────────────────
extern Servo            myServo;
extern SSD1306AsciiWire oled;

// ── Kalman filter instances ───────────────────────────────
extern Kalman kalmanRoll;
extern Kalman kalmanPitch;

// ── Calibration data ──────────────────────────────────────
extern CalibrationData calData;

// ── Servo ─────────────────────────────────────────────────
extern int currentServoAngle;   // 0-180 °

// ── IMU / heading ─────────────────────────────────────────
extern unsigned long lastTime;  // micros() timestamp for dt calculation
extern float kalRoll;
extern float kalPitch;
extern float magHeading;        // 0-360 ° tilt-compensated

// ── Motor & encoder ───────────────────────────────────────
extern int           motorSpeed;    // -255 … 255
extern volatile int32_t encoderCount;

// ── LED states ────────────────────────────────────────────
extern bool greenState;
extern bool yellowState;
extern bool redState;

// ── Button debounce ───────────────────────────────────────
extern bool          lastButton1State;
extern bool          lastButton2State;
extern unsigned long lastButton1Debounce;
extern unsigned long lastButton2Debounce;
extern volatile bool btn1Triggered;
extern volatile bool btn2Triggered;

// ── Non-blocking buzzer ───────────────────────────────────
extern unsigned long buzzerOffTime;

// ── Sensor health ─────────────────────────────────────
// false if MPU6050 or QMC5883L failed to init; guards IMU calls in loop().
extern bool sensorsOK;

// ── Odometry / Dead Reckoning ─────────────────────────────
extern float odomX;      // X position in mm
extern float odomY;      // Y position in mm
extern float odomDist;   // Total distance traveled in mm
extern float odomSpeed;  // Current speed in mm/s
extern float imuYaw;     // Fused IMU yaw heading in degrees (0-360)


