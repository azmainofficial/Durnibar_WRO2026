// =============================================================
// hardware_test.cpp  –  Full board self-test implementation.
// =============================================================
#include "hardware_test.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "peripherals.h"
#include "odometry.h"
#include "oled_display.h"


void runFullTest() {
  Serial.println("\n=== FULL HARDWARE TEST ===");
  oled.clear();
  oled.println("TESTING...");

  // ── 1. LEDs ─────────────────────────────────────────────
  oled.println("LEDs");
  digitalWrite(LED_GREEN_PIN,  HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  digitalWrite(LED_RED_PIN,    HIGH);
  delay(500);
  digitalWrite(LED_GREEN_PIN,  LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN,    LOW);
  Serial.println("LEDs: OK");

  // ── 2. Buzzer ────────────────────────────────────────────
  oled.println("Buzzer");
  // Use a blocking delay here – acceptable during a deliberate test sequence.
  digitalWrite(BUZZER_PIN, HIGH); delay(300); digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Buzzer: OK");

  // ── 3. Servo ─────────────────────────────────────────────
  oled.println("Servo");
  myServo.write(0);   delay(600);
  myServo.write(180); delay(600);
  myServo.write(90);  delay(300);
  Serial.println("Servo: OK");

  // ── 4. Motor ─────────────────────────────────────────────
  oled.println("Motor");
  setMotorSpeed(200);  delay(800);
  setMotorSpeed(-200); delay(800);
  setMotorSpeed(0);
  Serial.println("Motor: OK");

  // ── 5. Encoder ───────────────────────────────────────────
  int32_t enc = getEncoderCount();
  Serial.print("Encoder count: "); Serial.println(enc);
  oled.print("Enc:"); oled.println(enc);

  // ── 6. I²C sensors ───────────────────────────────────────
  oled.println("Sensors");
  bool mpuOK = initMPU6050();
  bool magOK = initMagnetometer();
  Serial.print("MPU6050: ");     Serial.println(mpuOK ? "OK" : "ERROR");
  Serial.print("Magnetometer: "); Serial.println(magOK ? "OK" : "ERROR");
  oled.print("MPU:"); oled.println(mpuOK ? "OK" : "FAIL");
  oled.print("MAG:"); oled.println(magOK ? "OK" : "FAIL");

  // ── 7. Buttons (5 s press window) ────────────────────────
  oled.println("Press B1/B2");
  Serial.println("Press Button 1 or 2 to test (timeout 5 s)");
  unsigned long start   = millis();
  bool          pressed = false;
  while (millis() - start < 5000) {
    if (digitalRead(BUTTON1_PIN) == LOW || digitalRead(BUTTON2_PIN) == LOW) {
      pressed = true;
      break;
    }
    delay(10);
  }
  Serial.println(pressed ? "Button test: OK" : "Button test: TIMEOUT");
  oled.println(pressed ? "Buttons OK" : "Buttons timeout");

  // ── 8. Live IMU (20 frames) ──────────────────────────────
  oled.println("Live data");
  for (int i = 0; i < 20; i++) {
    updateIMU();
    oled.home();
    oled.set2X();
    oled.print("R:"); oled.print((int)kalRoll);
    oled.print(" P:"); oled.print((int)kalPitch);
    oled.println();
    oled.print("H:"); oled.print((int)magHeading);
    oled.set1X();
    delay(100);
  }

  oled.clear();
  oled.println("Test Complete!");
  Serial.println("=== TEST COMPLETE ===");
  delay(1000);
  oled.clear();
}

void turnAngleTest(float turnAngleDeg, int speed) {
  Serial.println("\n=== TURN TEST ===");
  Serial.print("Target Angle: "); Serial.print(turnAngleDeg); Serial.println(" deg");
  Serial.print("Motor Speed: "); Serial.println(speed);

  oled.clear();
  oled.println("TURN TEST");
  oled.print("Angle:"); oled.print((int)turnAngleDeg); oled.print(" Spd:"); oled.println(speed);
  delay(1000);

  // Record initial starting yaw
  float startYaw = imuYaw;

  // Set steering angle
  // Positive angle = Turn Left (170°), Negative angle = Turn Right (50°)
  int steerAngle = STEER_SERVO_CENTER;
  if (turnAngleDeg > 0) {
    steerAngle = SERVO_LEFT_LIMIT;  // 170° servo angle for Left Turn
  } else {
    steerAngle = SERVO_RIGHT_LIMIT; // 50° servo angle for Right Turn
  }



  // Position steering servo
  currentServoAngle = steerAngle;
  myServo.write(currentServoAngle);
  delay(300);

  // Start motor
  setMotorSpeed(speed);

  // Monitor turn completion using IMU & Odometry
  unsigned long turnStartTime = millis();
  bool turnCompleted = false;

  while (millis() - turnStartTime < 10000) { // 10 second timeout safety
    updateIMU();
    updateOdometry();
    updateOLED();

    // Compute relative turned angle magnitude
    float turned = abs(imuYaw - startYaw);
    if (turned > 180.0f) turned = 360.0f - turned;

    // Apply early braking lead angle to compensate for vehicle momentum
    float stopThreshold = abs(turnAngleDeg) - TURN_BRAKE_LEAD_DEG;
    if (stopThreshold < 0.0f) stopThreshold = 0.0f;

    if (turned >= stopThreshold) {
      turnCompleted = true;
      break;
    }

    delay(10);
  }

  // Stop motor & center steering
  setMotorSpeed(0);
  currentServoAngle = STEER_SERVO_CENTER;
  myServo.write(currentServoAngle);

  oled.clear();
  if (turnCompleted) {
    Serial.println("Turn: COMPLETE!");
    oled.println("TURN COMPLETE!");
  } else {
    Serial.println("Turn: TIMEOUT!");
    oled.println("TURN TIMEOUT!");
  }
  delay(1500);
  oled.clear();
}

