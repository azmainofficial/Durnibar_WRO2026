// =============================================================
// Sulg.ino  –  Main sketch: global definitions + setup + loop.
//
// All feature code lives in separate header files:
//   config.h        – pin & address definitions
//   globals.h       – extern declarations for shared state
//   kalman.h        – Kalman filter class
//   calibration.h   – CalibrationData struct + EEPROM routines
//   imu.h           – MPU6050 + QMC5883L + sensor fusion
//   motor.h         – TB6612 motor driver
//   encoder.h       – PCNT quadrature encoder
//   peripherals.h   – LEDs, non-blocking buzzer, button callbacks
//   oled_display.h  – OLED status screen
//   serial_parser.h – Serial integer reader
//   hardware_test.h – Full board self-test
// =============================================================

// ── Library & feature includes ────────────────────────────
#include <ESP32Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include "config.h"
#include "kalman.h"          // must come before globals.h (defines Kalman class)
#include "globals.h"         // extern declarations – no storage here
#include "calibration.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "peripherals.h"
#include "oled_display.h"
#include "serial_parser.h"
#include "hardware_test.h"
#include "odometry.h"
#include "pi_comm.h"


// =============================================================
// GLOBAL DEFINITIONS
// (extern declarations are in globals.h; actual storage is here)
// =============================================================

// Hardware objects
Servo            myServo;
SSD1306AsciiWire oled;

// Kalman instances
Kalman kalmanRoll;
Kalman kalmanPitch;

// Calibration data (loaded from EEPROM in setup)
CalibrationData calData;

// Servo
int currentServoAngle = 110;

// IMU / heading
unsigned long lastTime   = 0;
float         kalRoll    = 0.0f;
float         kalPitch   = 0.0f;
float         magHeading = 0.0f;

// Odometry / Dead Reckoning variables
float odomX      = 0.0f;
float odomY      = 0.0f;
float odomDist   = 0.0f;
float odomSpeed  = 0.0f;
float imuYaw     = 0.0f;


// Motor & encoder
int              motorSpeed   = 0;
volatile int32_t encoderCount = 0;

// LED states
bool greenState  = false;
bool yellowState = false;
bool redState    = false;

// Button debounce
bool          lastButton1State    = HIGH;
bool          lastButton2State    = HIGH;
unsigned long lastButton1Debounce = 0;
unsigned long lastButton2Debounce = 0;
volatile bool btn1Triggered       = false;
volatile bool btn2Triggered       = false;

// Non-blocking buzzer
unsigned long buzzerOffTime = 0;

// Sensor health flag – set false if MPU6050 or magnetometer fails init.
// Prevents IMU-dependent code from running and crashing the loop.
bool sensorsOK = true;

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  // ── Buttons & boot button ─────────────────────────────
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN,     INPUT_PULLUP);
  pinMode(BUTTON2_PIN,     INPUT_PULLUP);

  // ── Buzzer & LEDs (first! so they always work) ────────
  pinMode(BUZZER_PIN,     OUTPUT); digitalWrite(BUZZER_PIN,     LOW);
  pinMode(LED_GREEN_PIN,  OUTPUT); digitalWrite(LED_GREEN_PIN,  LOW);
  pinMode(LED_YELLOW_PIN, OUTPUT); digitalWrite(LED_YELLOW_PIN, LOW);
  pinMode(LED_RED_PIN,    OUTPUT); digitalWrite(LED_RED_PIN,    LOW);

  // Quick sequential blink: proves all 3 LED pins are wired & working.
  digitalWrite(LED_GREEN_PIN,  HIGH); delay(120); digitalWrite(LED_GREEN_PIN,  LOW);
  digitalWrite(LED_YELLOW_PIN, HIGH); delay(120); digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN,    HIGH); delay(120); digitalWrite(LED_RED_PIN,    LOW);

  // ── Servo (attach BEFORE initMotor so it wins the first
  //    available LEDC channel/timer; motor gets the next one) ────
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(currentServoAngle);
  Serial.println("Servo: attached");

  // ── Motor & encoder ───────────────────────────────────
  initMotor();
  initEncoder();

  // ── EEPROM & calibration ──────────────────────────────
  EEPROM.begin(EEPROM_SIZE);
  loadCalibration(calData);

  // ── I²C bus ───────────────────────────────────────────
  Wire.begin(21, 22);           // SDA=21, SCL=22
  Wire.setClock(400000L);       // 400 kHz fast-mode

  // ── OLED ──────────────────────────────────────────────
  #if defined(OLED_TYPE) && (OLED_TYPE == SH1106)
    oled.begin(&SH1106_128x64, OLED_ADDR);
  #else
    oled.begin(&Adafruit128x64, OLED_ADDR);
  #endif
  oled.setFont(System5x7);
  oled.clear();

  // ── Sensor init (NON-FATAL: errors do NOT halt – servo, LEDs and motor keep working) ─────
  if (!initMPU6050()) {
    sensorsOK = false;
    oled.println("MPU6050 ERROR!");
    Serial.println("MPU6050 ERROR! IMU disabled.");
  }
  if (!initMagnetometer()) {
    sensorsOK = false;
    oled.println("MAG ERROR!");
    Serial.println("MAG ERROR! Heading disabled.");
  }

  // ── Seed Kalman filter dt timer ───────────────────────
  lastTime = micros();

  // ── Initialize Odometry ───────────────────────────────
  initOdometry();
  imuYaw = 0.0f; // Zero-heading reference at startup: car's initial facing direction is 0 degrees

  // ── Initialize Pi Communication ───────────────────────
  initPiComm();




  // ── Print command help ────────────────────────────────
  Serial.println("\n--- SYSTEM READY ---");
  Serial.print("  Sensors: "); Serial.println(sensorsOK ? "OK" : "ERROR (IMU disabled)");
  Serial.println("Commands:");
  Serial.println("  G/Y/L      - toggle Green / Yellow / Red LED");
  Serial.println("  B          - beep buzzer");
  Serial.println("  T          - full hardware test");
  Serial.println("  A          - 90 degree left turn test (speed 55)");

  Serial.println("  M<speed>   - set motor speed (-255..255)");
  Serial.println("  E          - read encoder count");
  Serial.println("  X          - reset encoder");
  Serial.println("  <0-180>    - move servo to angle");
  Serial.println("  B1 button  - 90 deg Left turn (speed 55)");
  Serial.println("  B2 button  - 90 deg Right turn (speed 55)");
  Serial.println("  BOOT button- calibrate IMU & magnetometer");


}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {

  // ── BOOT button (Disabled automatic calibration to prevent unwanted random freezing) ────
  // Calibration should only be invoked explicitly via serial command if requested.

  // ── BUTTON 1 (active-LOW) ───────────────────────────────
  {
    static bool button1ConfirmedState = HIGH;
    bool b1 = digitalRead(BUTTON1_PIN);
    if (b1 != lastButton1State) {
      lastButton1Debounce = millis();
      lastButton1State = b1;
    }
    if ((millis() - lastButton1Debounce) > DEBOUNCE_DELAY) {
      if (b1 != button1ConfirmedState) {
        button1ConfirmedState = b1;
        if (button1ConfirmedState == LOW) {
          handleButton1();
        }
      }
    }
  }

  // ── BUTTON 2 (active-LOW) ───────────────────────────────
  {
    static bool button2ConfirmedState = HIGH;
    bool b2 = digitalRead(BUTTON2_PIN);
    if (b2 != lastButton2State) {
      lastButton2Debounce = millis();
      lastButton2State = b2;
    }
    if ((millis() - lastButton2Debounce) > DEBOUNCE_DELAY) {
      if (b2 != button2ConfirmedState) {
        button2ConfirmedState = b2;
        if (button2ConfirmedState == LOW) {
          handleButton2();
        }
      }
    }
  }




  // ── Refresh encoder for OLED ──────────────────────────
  encoderCount = getEncoderCount();

  // ── Non-blocking buzzer tick ──────────────────────────
  updateBuzzer();

  // ── IMU sensor fusion (only if sensors initialised OK) ───
  if (sensorsOK) updateIMU();

  // ── Update Odometry ──────────────────────────────────────
  updateOdometry();

  // ── Raspberry Pi Communication & Telemetry Stream ────────
  updatePiComm();



  // ── OLED refresh @ 10 Hz ─────────────────────────────────
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 100) {
    updateOLED();
    lastDisplayUpdate = millis();
  }
}