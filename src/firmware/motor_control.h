/**
 * @file motor_control.h
 * @brief TB6612FNG Motor Driver & Ackermann Servo Steering Controller
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include <ESP32Servo.h>

static Servo g_steeringServo;

/**
 * @brief Initialize motor driver GPIOs and attach steering servo
 */
inline void initMotorAndServo() {
    pinMode(PIN_MOTOR_PWMA, OUTPUT);
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Drive STBY HIGH to enable TB6612FNG H-Bridge
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // Servo Setup
    ESP32PWM::allocateTimer(0);
    g_steeringServo.setPeriodHertz(50);
    g_steeringServo.attach(PIN_SERVO, 1000, 2000);
    g_steeringServo.write(SERVO_CENTER);
}

/**
 * @brief Set Rear Drive Motor Speed (-255 to 255)
 * @param speed Speed value: positive for forward, negative for reverse, 0 for stop
 */
inline void setMotorSpeed(int speed) {
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    if (speed > 0) { // Forward
        digitalWrite(PIN_MOTOR_AIN1, HIGH);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        analogWrite(PIN_MOTOR_PWMA, constrain(speed, 0, 255));
    } else if (speed < 0) { // Reverse
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, HIGH);
        analogWrite(PIN_MOTOR_PWMA, constrain(-speed, 0, 255));
    } else { // Active Brake
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        analogWrite(PIN_MOTOR_PWMA, 0);
    }
}

/**
 * @brief Set Steering Angle with Ackermann Protection
 * @param angle Target steering angle in degrees
 */
inline void setSteeringAngle(int angle) {
    int clampedAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    g_steeringServo.write(clampedAngle);
}

/**
 * @brief Emergency stop vehicle (Neutral steering + motor brake)
 */
inline void stopVehicle() {
    setMotorSpeed(0);
    setSteeringAngle(SERVO_CENTER);
}

#endif // MOTOR_CONTROL_H
