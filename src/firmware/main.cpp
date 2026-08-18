/**
 * @file main.cpp
 * @brief Modular Function-Based ESP32 Firmware for PlatformIO
 * @team Team Durnibar (Bangladesh)
 */

#include "config.h"
#include "kalman_filter.h"
#include "encoder.h"
#include "motor_control.h"
#include "sensors_display.h"

bool g_systemActive = false;
unsigned long g_lastTelemetryTime = 0;
unsigned long g_lastLoopTime = 0;

void setup() {
    Serial.begin(115200);

    initMotorAndServo();
    initEncoder();
    initSensorsAndDisplay();

    Serial.println("[ESP32] PlatformIO Modular Firmware Initialized!");
}

void parseIncomingCommands() {
    if (Serial.available() > 0) {
        String msg = Serial.readStringUntil('#');
        if (msg.startsWith("$CMD,")) {
            int comma1 = msg.indexOf(',');
            int comma2 = msg.indexOf(',', comma1 + 1);
            if (comma1 != -1 && comma2 != -1) {
                int angle = msg.substring(comma1 + 1, comma2).toInt();
                int speed = msg.substring(comma2 + 1).toInt();

                setSteeringAngle(angle);
                setMotorSpeed(speed);
            }
        }
    }
}

void sendTelemetryStream(float dt) {
    unsigned long now = millis();
    if (now - g_lastTelemetryTime >= 20) {
        g_lastTelemetryTime = now;

        float filteredGyroZ = 0.0f, filteredAccelX = 0.0f;
        getFilteredIMUData(filteredGyroZ, filteredAccelX);
        float filteredSpeed = getFilteredSpeedMmPerSec(dt);

        String telemetry = "$TEL," + String(getEncoderTicks()) + "," + 
                           String(filteredSpeed, 1) + "," + 
                           String(filteredGyroZ, 2) + "," + 
                           String(filteredAccelX, 2) + "," + 
                           String(g_systemActive ? 1 : 0) + "#";

        Serial.println(telemetry);
    }
}

void loop() {
    unsigned long currentMicros = micros();
    float dt = (currentMicros - g_lastLoopTime) / 1000000.0f;
    g_lastLoopTime = currentMicros;

    if (!g_systemActive && isStartButtonPressed()) {
        g_systemActive = true;
        setStatusLEDs(true, false, false);
        beepBuzzer(2000, 300);
        updateOLEDStatus("STATUS: RUNNING", "WRO 2026 Active");
        Serial.println("$EVENT,START_PRESSED#");
    }

    parseIncomingCommands();
    sendTelemetryStream(dt);
}
