// =============================================================
// config.h  –  Hardware pin definitions & compile-time constants
// Edit this file to adapt the sketch to different PCB layouts.
// =============================================================
#pragma once

// ── Servo ──────────────────────────────────────────────────
#define SERVO_PIN         18

// ── Buzzer & LEDs ──────────────────────────────────────────
#define BUZZER_PIN        19
#define LED_GREEN_PIN     16
#define LED_YELLOW_PIN    17
#define LED_RED_PIN        5

// ── Buttons (active LOW, external or internal pull-up) ─────
#define BOOT_BUTTON_PIN    0   // built-in BOOT button → triggers IMU calibration
#define BUTTON1_PIN       13   // toggles Red LED
#define BUTTON2_PIN       12   // toggles Green LED

// ── Motor driver (TB6612FNG) ───────────────────────────────
#define MOTOR_STBY_PIN    23
#define MOTOR_PWMA_PIN    25
#define MOTOR_AIN1_PIN    26
#define MOTOR_AIN2_PIN    27

// ── Quadrature encoder ─────────────────────────────────────
// NOTE: GPIO 34 & 35 are input-only with NO internal pull-up.
// Fit external 10 kΩ pull-up resistors to 3.3 V on both pins.
#define ENC_A_PIN         34
#define ENC_B_PIN         35

// ── I²C peripheral addresses ──────────────────────────────
#define MPU6050_ADDR      0x68
#define QMC5883L_ADDR     0x0D
#define OLED_ADDR         0x3C

// ── EEPROM ────────────────────────────────────────────────
#define EEPROM_SIZE       128

// ── OLED panel type: SSD1306 (default) or SH1106 ─────────
#define OLED_TYPE         SSD1306

// ── Button debounce window (ms) ───────────────────────────
#define DEBOUNCE_DELAY    50UL

// ── IMU Sensor Calibration ────────────────────────────────
// Set INVERT_GYRO_Z to true if MPU6050 is mounted upside down
#define INVERT_GYRO_Z     false


// ── Physical Vehicle & Wheel Geometry ──────────────────────
#define WHEEL_DIAMETER_MM      53.4f
#define WHEEL_CIRCUMFERENCE_MM 167.76f
#define WHEELBASE_MM           200.0f  // Distance between front & rear axles (mm)
#define TRACK_WIDTH_MM         200.0f  // Distance between left & right wheels (mm)

// ── Motor Driver Calibration ───────────────────────────────
// Set INVERT_MOTOR_DIR to true if positive motor speed drives the vehicle backward
#define INVERT_MOTOR_DIR       true

// ── Spur Gear & Encoder Calibration ────────────────────────
// Set ENCODER_INVERT_DIR to true if 2 spur gears reverse the rotation direction
#define ENCODER_INVERT_DIR     true


// Gear ratio between encoder shaft and wheel axle (Teeth_wheel / Teeth_encoder)
// Or calibrated scale factor: 609.6 mm actual / 170 mm measured = ~3.58588
#define DRIVE_GEAR_RATIO       3.58588f

// Base encoder resolution per motor shaft revolution
#define ENCODER_CPR            360.0f  

// Effective CPR at wheel: ENCODER_CPR / DRIVE_GEAR_RATIO
#define EFFECTIVE_ENCODER_CPR  (ENCODER_CPR / DRIVE_GEAR_RATIO)

// ── Steering Calibration ───────────────────────────────────
#define STEER_SERVO_CENTER     110     // Servo angle (degrees) for driving straight
#define SERVO_LEFT_LIMIT       170     // Servo angle for Left turn (20cm forward, 20cm left)
#define SERVO_RIGHT_LIMIT      50      // Servo angle for Right turn (20cm forward, 20cm right)
#define STEER_MAX_ANGLE_DEG    45.0f   // Physical wheel steer angle (degrees)

// ── Turn Control & Momentum Compensation ────────────────────
// Early braking lead angle (degrees) to stop motor before target heading so vehicle momentum lands exactly on target
#define TURN_BRAKE_LEAD_DEG    8.5f






