/**
 * @file config.h
 * @brief Central Hardware Configuration & Pin Definitions for Team Durnibar ESP32
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// PIN MAPPING (User Specification)
// ==========================================
// TB6612FNG Motor Driver
#define PIN_MOTOR_PWMA  25
#define PIN_MOTOR_AIN1  26
#define PIN_MOTOR_AIN2  27
#define PIN_MOTOR_STBY  23

// Quadrature Wheel Encoder
#define PIN_ENCODER_A   34 // Interrupt pin (Input Only)
#define PIN_ENCODER_B   35 // Input Only

// Steering Servo
#define PIN_SERVO       18

// Audio / Visual Indicators
#define PIN_BUZZER      19
#define PIN_LED_GREEN   16
#define PIN_LED_YELLOW  17
#define PIN_LED_RED     5

// Push Buttons (INPUT_PULLUP)
#define PIN_BUTTON_1    13 // WRO 2026 Rule 9.11 Start Button
#define PIN_BUTTON_2    12
#define PIN_BUTTON_3    14

// I2C Pins & Display
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64

// ==========================================
// PHYSICAL & SAFETY LIMITS
// ==========================================
#define SERVO_CENTER      90
#define SERVO_MIN_ANGLE   62   // Max Right Turn
#define SERVO_MAX_ANGLE   118  // Max Left Turn

#define WHEEL_DIAMETER_MM 65.0f
#define TICKS_PER_REV     480.0f
#define MM_PER_TICK       ((3.14159f * WHEEL_DIAMETER_MM) / TICKS_PER_REV)

#endif // CONFIG_H
