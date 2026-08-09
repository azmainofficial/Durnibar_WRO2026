/**
 * @file encoder.h
 * @brief Wheel Encoder Driver & Odometry Functions
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"
#include "kalman_filter.h"

static volatile long g_encoderTicks = 0;
static KalmanFilter g_speedFilter(0.02f, 0.2f, 1.0f, 0.0f);

// Interrupt Service Routine for Encoder A
void IRAM_ATTR encoderISR() {
    if (digitalRead(PIN_ENCODER_B) == HIGH) {
        g_encoderTicks++;
    } else {
        g_encoderTicks--;
    }
}

/**
 * @brief Initialize encoder GPIO pins and attach interrupt
 */
inline void initEncoder() {
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, RISING);
}

/**
 * @brief Get total accumulated encoder ticks
 */
inline long getEncoderTicks() {
    return g_encoderTicks;
}

/**
 * @brief Reset encoder tick counter to zero
 */
inline void resetEncoderTicks() {
    g_encoderTicks = 0;
    g_speedFilter.reset(0.0f);
}

/**
 * @brief Calculate linear distance driven in millimeters
 */
inline float getDistanceDrivenMm() {
    return g_encoderTicks * MM_PER_TICK;
}

/**
 * @brief Calculate Kalman-filtered linear speed in mm/s
 * @param dt Delta time in seconds
 */
inline float getFilteredSpeedMmPerSec(float dt) {
    static long lastTicks = 0;
    if (dt <= 0.0001f) return 0.0f;

    long deltaTicks = g_encoderTicks - lastTicks;
    lastTicks = g_encoderTicks;

    float rawSpeed = (deltaTicks * MM_PER_TICK) / dt;
    return g_speedFilter.update(rawSpeed);
}

#endif // ENCODER_H
