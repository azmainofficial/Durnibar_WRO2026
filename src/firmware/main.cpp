/**
 * @file main.cpp
 * @brief ESP32 Hardware Firmware for Team Durnibar WRO 2026 Self-Driving Car
 * @team Team Durnibar (Bangladesh)
 * 
 * Hardware Pin Diagram (User Specification):
 * - Motor Driver (TB6612FNG): PWMA=25, AIN1=26, AIN2=27, STBY=23
 * - Quadrature Encoder:      Encoder A=34 (Interrupt), Encoder B=35
 * - Steering Servo:          Servo=18 (PWM 50Hz)
 * - Status Audio/Visual:     Buzzer=19, LED Green=16, LED Yellow=17, LED Red=5
 * - User Buttons:            Button 1 (Start)=13, Button 2=12, Button 3=14 (INPUT_PULLUP)
 * - Shared I2C Bus:          SDA=21, SCL=22 (OLED SSD1306, MPU6050, QMC5883L)
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
// TB6612FNG Motor Driver Pins
#define PIN_MOTOR_PWMA  25
#define PIN_MOTOR_AIN1  26
#define PIN_MOTOR_AIN2  27
#define PIN_MOTOR_STBY  23

// Quadrature Wheel Encoder Pins
#define PIN_ENCODER_A   34
#define PIN_ENCODER_B   35

// Steering Servo Pin
#define PIN_SERVO       18

// Status Audio/Visual Pins
#define PIN_BUZZER      19
#define PIN_LED_GREEN   16
#define PIN_LED_YELLOW  17
#define PIN_LED_RED     5

// Push Buttons (INPUT_PULLUP)
#define PIN_BUTTON_1    13 // WRO 2026 Rule 9.11 Start Button
#define PIN_BUTTON_2    12
#define PIN_BUTTON_3    14

// I2C Pins & Screen Spec
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64

// Steering Angle Safety Limits
#define SERVO_CENTER      90
#define SERVO_MIN_ANGLE   62   // Max Right Turn
#define SERVO_MAX_ANGLE   118  // Max Left Turn

// ==========================================
// 2. GLOBAL OBJECTS & VARIABLES
// ==========================================
Servo steeringServo;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

volatile long encoderTicks = 0;
bool isSystemActive = false;
unsigned long lastTelemetryTime = 0;

// Interrupt Service Routine for Wheel Encoder
void IRAM_ATTR encoderISR() {
    if (digitalRead(PIN_ENCODER_B) == HIGH) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
}

// ==========================================
// 3. MOTOR CONTROL HELPER (TB6612FNG)
// ==========================================
void setMotorSpeed(int speed) {
    // Enable Motor Driver
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    if (speed > 0) { // Forward
        digitalWrite(PIN_MOTOR_AIN1, HIGH);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        analogWrite(PIN_MOTOR_PWMA, constrain(speed, 0, 255));
    } else if (speed < 0) { // Reverse
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, HIGH);
        analogWrite(PIN_MOTOR_PWMA, constrain(-speed, 0, 255));
    } else { // Brake / Stop
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        analogWrite(PIN_MOTOR_PWMA, 0);
    }
}

// ==========================================
// 4. AUDIO / VISUAL INDICATOR HELPER
// ==========================================
void setStatusLEDs(bool green, bool yellow, bool red) {
    digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
    digitalWrite(PIN_LED_RED, red ? HIGH : LOW);
}

void beepBuzzer(int frequency, int durationMs) {
    tone(PIN_BUZZER, frequency, durationMs);
}

// ==========================================
// 5. SETUP FUNCTION
// ==========================================
void setup() {
    Serial.begin(115200);

    // Initialize Motor Driver Pins
    pinMode(PIN_MOTOR_PWMA, OUTPUT);
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, HIGH);
    setMotorSpeed(0);

    // Initialize Status Indicators
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    setStatusLEDs(false, true, false); // Booting (Yellow LED)

    // Initialize Buttons
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    pinMode(PIN_BUTTON_3, INPUT_PULLUP);

    // Initialize Encoder Interrupt
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, RISING);

    // Initialize Steering Servo
    ESP32PWM::allocateTimer(0);
    steeringServo.setPeriodHertz(50);
    steeringServo.attach(PIN_SERVO, 1000, 2000);
    steeringServo.write(SERVO_CENTER);

    // Initialize I2C Bus & OLED Display
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("TEAM DURNIBAR 2026");
        display.println("ESP32 System Ready");
        display.println("Press Button 1 to Start");
        display.display();
    }

    // Initialize MPU6050
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    setStatusLEDs(false, false, true); // Ready state (Red standby LED)
    beepBuzzer(1000, 150);
    Serial.println("[ESP32] Hardware Setup Complete. Waiting for Button 1...");
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
    // 1. Read Button 1 (WRO 2026 Rule 9.11 Start Button)
    if (digitalRead(PIN_BUTTON_1) == LOW && !isSystemActive) {
        delay(50); // Debounce
        if (digitalRead(PIN_BUTTON_1) == LOW) {
            isSystemActive = true;
            setStatusLEDs(true, false, false); // Active state (Green LED)
            beepBuzzer(2000, 300);

            if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.println("STATUS: RUNNING");
                display.println("WRO 2026 Active");
                display.display();
            }

            Serial.println("$EVENT,START_PRESSED#");
        }
    }

    // 2. Parse Incoming Control Commands from Serial
    // Format: "$CMD,<SteeringAngle>,<MotorSpeed PWM -255 to 255>#"
    if (Serial.available() > 0) {
        String msg = Serial.readStringUntil('#');
        if (msg.startsWith("$CMD,")) {
            int comma1 = msg.indexOf(',');
            int comma2 = msg.indexOf(',', comma1 + 1);
            if (comma1 != -1 && comma2 != -1) {
                int angle = msg.substring(comma1 + 1, comma2).toInt();
                int speed = msg.substring(comma2 + 1).toInt();

                steeringServo.write(constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE));
                setMotorSpeed(speed);
            }
        }
    }

    // 3. Telemetry Stream at 50 Hz
    unsigned long now = millis();
    if (now - lastTelemetryTime >= 20) {
        lastTelemetryTime = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Build Telemetry: "$TEL,<Ticks>,<GyroZ>,<AccelX>,<Active>#"
        String telemetry = "$TEL," + String(encoderTicks) + "," + 
                           String(g.gyro.z, 2) + "," + 
                           String(a.acceleration.x, 2) + "," + 
                           String(isSystemActive ? 1 : 0) + "#";

        Serial.println(telemetry);
    }
}
