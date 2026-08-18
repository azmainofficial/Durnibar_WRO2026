// =============================================================
// motor.cpp  –  DC motor driver implementation.
// =============================================================
#include "motor.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"

// ── Initialise motor GPIO & PWM ───────────────────────────
void initMotor() {
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);

  digitalWrite(MOTOR_STBY_PIN, HIGH);  // release standby
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);

  // ESP32 Arduino v3+: ledcAttach(pin, freq, resolution_bits)
  ledcAttach(MOTOR_PWMA_PIN, 20000, 8);  // 20 kHz, 8-bit (0-255)
  ledcWrite(MOTOR_PWMA_PIN, 0);
}

// ── Set motor speed ───────────────────────────────────────
// speed > 0  → forward   (AIN1 HIGH, AIN2 LOW)
// speed < 0  → reverse   (AIN1 LOW,  AIN2 HIGH)
// speed == 0 → brake     (AIN1 HIGH, AIN2 HIGH, PWM=0)
// TB6612 coast = both LOW; active brake = both HIGH (we use brake).
void setMotorSpeed(int speed) {
  motorSpeed = constrain(speed, -255, 255);

  int driveSpeed = motorSpeed;
#if defined(INVERT_MOTOR_DIR) && (INVERT_MOTOR_DIR == true)
  driveSpeed = -driveSpeed;
#endif

  if (driveSpeed > 0) {
    digitalWrite(MOTOR_AIN1_PIN, HIGH);
    digitalWrite(MOTOR_AIN2_PIN, LOW);
    ledcWrite(MOTOR_PWMA_PIN, (uint8_t)driveSpeed);
  } else if (driveSpeed < 0) {
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, HIGH);
    ledcWrite(MOTOR_PWMA_PIN, (uint8_t)(-driveSpeed));
  } else {
    // Active brake: both AIN HIGH, PWM = 0
    digitalWrite(MOTOR_AIN1_PIN, HIGH);
    digitalWrite(MOTOR_AIN2_PIN, HIGH);
    ledcWrite(MOTOR_PWMA_PIN, 0);
  }
}

