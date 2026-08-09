/**
 * @file main.cpp
 * @brief Low-Level ESP32 Firmware for Team Durnibar WRO 2026 Self-Driving Vehicle
 * @team Team Durnibar (Bangladesh)
 * 
 * Hardware Mapping:
 * - Steering Servo: GPIO 18 (PWM 50Hz)
 * - Drive Motor ESC: GPIO 19 (PWM 50Hz)
 * - Start Button:    GPIO 4 (Internal Pull-Up - Rule 9.11)
 * - UART to RPi 5:   GPIO 16 (RX2), GPIO 17 (TX2) @ 115200 Baud
 * - I2C Bus:         GPIO 21 (SDA), GPIO 22 (SCL) @ 400kHz
 * - ToF XSHUT Pins:  GPIO 25 (FL), GPIO 26 (FR), GPIO 27 (SL), GPIO 14 (SR)
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BNO055.h>

// ==========================================
// 1. PIN DEFINITIONS & CONSTANTS
// ==========================================
#define PIN_SERVO         18
#define PIN_ESC           19
#define PIN_START_BUTTON  4

#define PIN_XSHUT_FL      25
#define PIN_XSHUT_FR      26
#define PIN_XSHUT_SL      27
#define PIN_XSHUT_SR      14

#define I2C_ADDR_TOF_FL   0x30
#define I2C_ADDR_TOF_FR   0x31
#define I2C_ADDR_TOF_SL   0x32
#define I2C_ADDR_TOF_SR   0x33

// Servo Angle Limits (Ackermann Geometry Protection)
#define SERVO_CENTER      90
#define SERVO_MIN_ANGLE   62   // Max Right Turn
#define SERVO_MAX_ANGLE   118  // Max Left Turn

// ESC PWM Limits (Neutral = 1500us)
#define ESC_NEUTRAL_US    1500
#define ESC_MIN_US        1100 // Max Reverse
#define ESC_MAX_US        1900 // Max Forward

// ==========================================
// 2. GLOBAL OBJECTS
// ==========================================
Servo steeringServo;
Servo motorESC;

Adafruit_VL53L1X tofFL = Adafruit_VL53L1X(PIN_XSHUT_FL);
Adafruit_VL53L1X tofFR = Adafruit_VL53L1X(PIN_XSHUT_FR);
Adafruit_VL53L1X tofSL = Adafruit_VL53L1X(PIN_XSHUT_SL);
Adafruit_VL53L1X tofSR = Adafruit_VL53L1X(PIN_XSHUT_SR);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool systemStarted = false;
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50 Hz telemetry stream to RPi 5

// ==========================================
// 3. TOF SENSOR INITIALIZATION (I2C ADDR ASSIGNMENT)
// ==========================================
void initToFSensors() {
    // Reset all ToF sensors using XSHUT pins
    pinMode(PIN_XSHUT_FL, OUTPUT); digitalWrite(PIN_XSHUT_FL, LOW);
    pinMode(PIN_XSHUT_FR, OUTPUT); digitalWrite(PIN_XSHUT_FR, LOW);
    pinMode(PIN_XSHUT_SL, OUTPUT); digitalWrite(PIN_XSHUT_SL, LOW);
    pinMode(PIN_XSHUT_SR, OUTPUT); digitalWrite(PIN_XSHUT_SR, LOW);
    delay(10);

    // Initialize Front-Left ToF
    digitalWrite(PIN_XSHUT_FL, HIGH); delay(10);
    if (tofFL.begin(I2C_ADDR_TOF_FL, &Wire)) {
        tofFL.startRanging();
        tofFL.setTimingBudget(20);
    }

    // Initialize Front-Right ToF
    digitalWrite(PIN_XSHUT_FR, HIGH); delay(10);
    if (tofFR.begin(I2C_ADDR_TOF_FR, &Wire)) {
        tofFR.startRanging();
        tofFR.setTimingBudget(20);
    }

    // Initialize Side-Left ToF
    digitalWrite(PIN_XSHUT_SL, HIGH); delay(10);
    if (tofSL.begin(I2C_ADDR_TOF_SL, &Wire)) {
        tofSL.startRanging();
        tofSL.setTimingBudget(20);
    }

    // Initialize Side-Right ToF
    digitalWrite(PIN_XSHUT_SR, HIGH); delay(10);
    if (tofSR.begin(I2C_ADDR_TOF_SR, &Wire)) {
        tofSR.startRanging();
        tofSR.setTimingBudget(20);
    }
}

// ==========================================
// 4. SETUP FUNCTION
// ==========================================
void setup() {
    // Serial debugging console
    Serial.begin(115200);

    // High-Speed Serial UART to Raspberry Pi 5 (GPIO 16 = RX, GPIO 17 = TX)
    Serial2.begin(115200, SERIAL_8N1, 16, 17);

    // I2C Bus setup (400kHz fast mode)
    Wire.begin(21, 22);
    Wire.setClock(400000);

    // Start Button (WRO 2026 Rule 9.11)
    pinMode(PIN_START_BUTTON, INPUT_PULLUP);

    // Attach Servo & ESC
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    steeringServo.setPeriodHertz(50);
    motorESC.setPeriodHertz(50);
    
    steeringServo.attach(PIN_SERVO, 1000, 2000);
    motorESC.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);

    // Set Neutral initial position
    steeringServo.write(SERVO_CENTER);
    motorESC.writeMicroseconds(ESC_NEUTRAL_US);

    // Init Sensors
    initToFSensors();
    bno.begin();

    Serial.println("[ESP32] Hardware Init Complete. Waiting for Start Button / RPi 5 Serial...");
}

// ==========================================
// 5. SERIAL COMMAND PARSER FROM RPI 5
// Format: "$CMD,<SteeringAngle>,<ThrottleUs>#"
// Example: "$CMD,90,1550#"
// ==========================================
void handleIncomingSerialCommands() {
    while (Serial2.available() > 0) {
        String msg = Serial2.readStringUntil('#');
        if (msg.startsWith("$CMD,")) {
            int firstComma = msg.indexOf(',');
            int secondComma = msg.indexOf(',', firstComma + 1);
            if (firstComma != -1 && secondComma != -1) {
                int targetSteering = msg.substring(firstComma + 1, secondComma).toInt();
                int targetThrottle = msg.substring(secondComma + 1).toInt();

                // Constrain for safety
                targetSteering = constrain(targetSteering, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
                targetThrottle = constrain(targetThrottle, ESC_MIN_US, ESC_MAX_US);

                steeringServo.write(targetSteering);
                motorESC.writeMicroseconds(targetThrottle);
            }
        }
    }
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
    // 1. Read Start Push Button (Rule 9.11)
    if (digitalRead(PIN_START_BUTTON) == LOW && !systemStarted) {
        delay(50); // Debounce
        if (digitalRead(PIN_START_BUTTON) == LOW) {
            systemStarted = true;
            Serial2.println("$EVENT,START_PRESSED#");
            Serial.println("[ESP32] Start Button Pressed! System Active.");
        }
    }

    // 2. Parse Control Commands from RPi 5
    handleIncomingSerialCommands();

    // 3. Send Telemetry Stream to RPi 5 at 50 Hz
    unsigned long now = millis();
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = now;

        // Read Distance Sensors (mm)
        int16_t distFL = tofFL.dataReady() ? tofFL.distance() : -1;
        int16_t distFR = tofFR.dataReady() ? tofFR.distance() : -1;
        int16_t distSL = tofSL.dataReady() ? tofSL.distance() : -1;
        int16_t distSR = tofSR.dataReady() ? tofSR.distance() : -1;

        // Read IMU Orientation (Degrees)
        sensors_event_t event;
        bno.getEvent(&event);
        float yaw = event.orientation.x;

        // Build Telemetry String: "$TEL,<FL>,<FR>,<SL>,<SR>,<Yaw>,<BtnState>#"
        String telemetry = "$TEL," + String(distFL) + "," + String(distFR) + "," + 
                           String(distSL) + "," + String(distSR) + "," + 
                           String(yaw, 1) + "," + String(systemStarted ? 1 : 0) + "#";

        Serial2.println(telemetry);
    }
}
