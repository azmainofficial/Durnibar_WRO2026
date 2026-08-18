// =============================================================
// pi_comm.cpp  –  Raspberry Pi 5 Communication & Telemetry.
// =============================================================
#include "pi_comm.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "motor.h"
#include "odometry.h"
#include "hardware_test.h"
#include "peripherals.h"
#include "encoder.h"

static unsigned long lastTelemetryTime = 0;

void initPiComm() {
  lastTelemetryTime = millis();
}

void updatePiComm() {
  // ── 1. Process Incoming Commands from Raspberry Pi or Serial Terminal ─────
  while (Serial.available() > 0) {
    char cmd = Serial.peek();

    // Ignore leading whitespace/newline characters
    if (cmd == ' ' || cmd == '\r' || cmd == '\n' || cmd == '\t') {
      Serial.read();
      continue;
    }

    // Drive Command: D <speed> <steer_angle> (e.g. "D 130 110")
    if (cmd == 'D' || cmd == 'd') {
      Serial.read(); // Consume command character
      int speed = Serial.parseInt();
      int steer = Serial.parseInt();

      if (speed >= -255 && speed <= 255) {
        setMotorSpeed(speed);
      }
      if (steer >= 0 && steer <= 180) {
        currentServoAngle = steer;
        myServo.write(currentServoAngle);
      }
    }
    // Stop Immediately Command: S
    else if (cmd == 'S' || cmd == 's') {
      Serial.read();
      setMotorSpeed(0);
      currentServoAngle = STEER_SERVO_CENTER;
      myServo.write(currentServoAngle);
    }
    // Reset Odometry Command: R
    else if (cmd == 'R' || cmd == 'r') {
      Serial.read();
      initOdometry();
      imuYaw = 0.0f;
    }
    // Green LED: G or G <0|1>
    else if (cmd == 'G' || cmd == 'g') {
      Serial.read();
      char next = Serial.peek();
      while (next == ' ' || next == '\t') { Serial.read(); next = Serial.peek(); }
      if (next == '0' || next == '1') {
        int v = Serial.parseInt();
        setLED(LED_GREEN_PIN, greenState, v == 1);
      } else {
        toggleLED(LED_GREEN_PIN, greenState);
      }
    }
    // Yellow LED: Y or Y <0|1>
    else if (cmd == 'Y' || cmd == 'y') {
      Serial.read();
      char next = Serial.peek();
      while (next == ' ' || next == '\t') { Serial.read(); next = Serial.peek(); }
      if (next == '0' || next == '1') {
        int v = Serial.parseInt();
        setLED(LED_YELLOW_PIN, yellowState, v == 1);
      } else {
        toggleLED(LED_YELLOW_PIN, yellowState);
      }
    }
    // Red LED: L or L <0|1>
    else if (cmd == 'L' || cmd == 'l') {
      Serial.read();
      char next = Serial.peek();
      while (next == ' ' || next == '\t') { Serial.read(); next = Serial.peek(); }
      if (next == '0' || next == '1') {
        int v = Serial.parseInt();
        setLED(LED_RED_PIN, redState, v == 1);
      } else {
        toggleLED(LED_RED_PIN, redState);
      }
    }
    // Beep Buzzer: B
    else if (cmd == 'B' || cmd == 'b') {
      Serial.read();
      beepBuzzer(200);
    }
    // Interactive Motor Speed: M <speed> (e.g. "M 100")
    else if (cmd == 'M' || cmd == 'm') {
      Serial.read();
      int speed = Serial.parseInt();
      if (speed >= -255 && speed <= 255) {
        setMotorSpeed(speed);
        Serial.print("Motor speed set to "); Serial.println(speed);
      } else {
        Serial.println("Invalid speed (-255..255)");
      }
    }
    // Read Encoder Count: E
    else if (cmd == 'E' || cmd == 'e') {
      Serial.read();
      Serial.print("Encoder: "); Serial.println(getEncoderCount());
    }
    // Reset Encoder Count: X
    else if (cmd == 'X' || cmd == 'x') {
      Serial.read();
      resetEncoder();
      Serial.println("Encoder reset");
    }
    // Direct Numeric Servo Angle: e.g. "110"
    else if (cmd >= '0' && cmd <= '9') {
      int angle = Serial.parseInt();
      if (angle >= 0 && angle <= 180) {
        currentServoAngle = angle;
        myServo.write(currentServoAngle);
        Serial.print("Servo set to "); Serial.println(angle);
      }
    }
    // Unrecognized Command: Consume character to avoid lockups
    else {
      Serial.read();
    }
  }

  // ── 2. Stream Telemetry to Raspberry Pi 5 at 20 Hz (every 50 ms) ─
  unsigned long now = millis();
  if (now - lastTelemetryTime >= 50) {
    lastTelemetryTime = now;

    // Telemetry Format: ODOM,X_mm,Y_mm,Yaw_deg,Dist_mm,Speed_mms,SensorsOK,Btn1,Btn2
    Serial.print("ODOM,");
    Serial.print((int)odomX);         Serial.print(",");
    Serial.print((int)odomY);         Serial.print(",");
    Serial.print(imuYaw, 1);          Serial.print(",");
    Serial.print((int)odomDist);      Serial.print(",");
    Serial.print((int)odomSpeed);     Serial.print(",");
    Serial.print(sensorsOK ? 1 : 0);  Serial.print(",");
    Serial.print(btn1Triggered ? 1 : 0); Serial.print(",");
    Serial.println(btn2Triggered ? 1 : 0);

    // Clear triggers
    btn1Triggered = false;
    btn2Triggered = false;
  }
}
