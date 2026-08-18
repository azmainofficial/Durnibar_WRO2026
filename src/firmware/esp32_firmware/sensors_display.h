/**
 * @file sensors_display.h
 * @brief MPU6050 IMU, SSD1306 OLED Display, LEDs, Buzzer, & Button Driver
 */

#ifndef SENSORS_DISPLAY_H
#define SENSORS_DISPLAY_H

#include "config.h"
#include "kalman_filter.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

static Adafruit_MPU6050 g_mpu;
static Adafruit_SSD1306 g_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static KalmanFilter g_gyroZFilter(0.01f, 0.1f, 1.0f, 0.0f);
static KalmanFilter g_accelXFilter(0.05f, 0.3f, 1.0f, 0.0f);

/**
 * @brief Set status LED outputs
 */
inline void setStatusLEDs(bool green, bool yellow, bool red) {
    digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
    digitalWrite(PIN_LED_RED, red ? HIGH : LOW);
}

/**
 * @brief Emit tone pulse on status buzzer
 */
inline void beepBuzzer(int frequency, int durationMs) {
    tone(PIN_BUZZER, frequency, durationMs);
}

/**
 * @brief Initialize Status LEDs, Buzzer, Buttons, I2C, OLED, and MPU6050
 */
inline void initSensorsAndDisplay() {
    // LED Pins
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    setStatusLEDs(false, true, false); // Booting (Yellow LED)

    // Push Buttons
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    pinMode(PIN_BUTTON_3, INPUT_PULLUP);

    // I2C Bus & OLED
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    
    if (g_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        g_display.clearDisplay();
        g_display.setTextSize(1);
        g_display.setTextColor(SSD1306_WHITE);
        g_display.setCursor(0, 0);
        g_display.println("TEAM DURNIBAR 2026");
        g_display.println("ESP32 System Ready");
        g_display.println("Press Button 1 to Start");
        g_display.display();
    }

    // MPU6050 Setup
    if (g_mpu.begin()) {
        g_mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        g_mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        g_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    setStatusLEDs(false, false, true); // Ready state (Red standby LED)
    beepBuzzer(1000, 150);
}

/**
 * @brief Check if Button 1 (WRO Rule 9.11 Start Button) is pressed
 */
inline bool isStartButtonPressed() {
    if (digitalRead(PIN_BUTTON_1) == LOW) {
        delay(50); // Debounce
        return (digitalRead(PIN_BUTTON_1) == LOW);
    }
    return false;
}

/**
 * @brief Update OLED status text
 */
inline void updateOLEDStatus(const char* statusText, const char* detailText) {
    g_display.clearDisplay();
    g_display.setCursor(0, 0);
    g_display.setTextSize(1);
    g_display.println("TEAM DURNIBAR 2026");
    g_display.println(statusText);
    g_display.println(detailText);
    g_display.display();
}

/**
 * @brief Read MPU6050 sensors and return Kalman-filtered Gyro Z & Accel X
 */
inline void getFilteredIMUData(float &filteredGyroZ, float &filteredAccelX) {
    sensors_event_t a, g, temp;
    g_mpu.getEvent(&a, &g, &temp);

    filteredGyroZ = g_gyroZFilter.update(g.gyro.z);
    filteredAccelX = g_accelXFilter.update(a.acceleration.x);
}

#endif // SENSORS_DISPLAY_H
