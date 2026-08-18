/**
 * @file esp32_firmware.ino
 * @brief Modular Function-Based ESP32 Sketch for Team Durnibar WRO 2026 Vehicle
 * @team Team Durnibar (Bangladesh)
 * 
 * Modular Headers:
 * - config.h: Central Pinouts & Parameters
 * - motor_control.h: TB6612FNG Motor & Steering Servo Functions
 * - encoder.h: Quadrature Wheel Encoder ISR & Odometry
 * - kalman_filter.h: 1D Discrete Kalman Filter
 * - sensors_display.h: MPU6050 IMU, OLED, LEDs, Buzzer, & Button Logic
 */

#include "config.h"
#include "kalman_filter.h"
#include "encoder.h"
#include "motor_control.h"
#include "sensors_display.h"

// System State Variables
bool g_systemActive = false;
unsigned long g_lastTelemetryTime = 0;
unsigned long g_lastLoopTime = 0;

// ==========================================
// SETUP FUNCTION
// ==========================================
void setup() {
    // 1. Initialize Serial Communication
    Serial.begin(115200);

    // 2. Initialize Hardware Subsystems
    initMotorAndServo();
    initEncoder();
    initSensorsAndDisplay();

    Serial.println("[ESP32] All Modular Subsystems Initialized Successfully!");
}

// ==========================================
// SERIAL COMMAND PARSER
// Format: "$CMD,<SteeringAngle>,<MotorSpeed PWM -255 to 255>#"
// ==========================================
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

// ==========================================
// TELEMETRY TRANSMISSION (50 Hz)
// ==========================================
void sendTelemetryStream(float dt) {
    unsigned long now = millis();
    if (now - g_lastTelemetryTime >= 20) { // 50 Hz interval
        g_lastTelemetryTime = now;

        // Read Filtered IMU & Encoder Speed
        float filteredGyroZ = 0.0f, filteredAccelX = 0.0f;
        getFilteredIMUData(filteredGyroZ, filteredAccelX);
        float filteredSpeed = getFilteredSpeedMmPerSec(dt);

        // Build Telemetry: "$TEL,<Ticks>,<SpeedMmS>,<GyroZ>,<AccelX>,<ActiveState>#"
        String telemetry = "$TEL," + String(getEncoderTicks()) + "," + 
                           String(filteredSpeed, 1) + "," + 
                           String(filteredGyroZ, 2) + "," + 
                           String(filteredAccelX, 2) + "," + 
                           String(g_systemActive ? 1 : 0) + "#";

        Serial.println(telemetry);
    }
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    // Delta time calculation for Kalman speed estimation
    unsigned long currentMicros = micros();
    float dt = (currentMicros - g_lastLoopTime) / 1000000.0f;
    g_lastLoopTime = currentMicros;

    // 1. Check Button 1 (WRO 2026 Rule 9.11 Start Button)
    if (!g_systemActive && isStartButtonPressed()) {
        g_systemActive = true;
        setStatusLEDs(true, false, false); // Green LED Active
        beepBuzzer(2000, 300);
        updateOLEDStatus("STATUS: RUNNING", "WRO 2026 Active");
        Serial.println("$EVENT,START_PRESSED#");
    }

    // 2. Parse Commands from Raspberry Pi 5
    parseIncomingCommands();

    // 3. Send Kalman-Filtered Telemetry Stream
    sendTelemetryStream(dt);
}
