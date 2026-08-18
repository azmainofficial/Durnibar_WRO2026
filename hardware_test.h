// =============================================================
// hardware_test.h  –  Full board self-test (triggered by 'T' command).
//
// Exercises every peripheral in sequence and reports PASS/FAIL
// to both Serial and the OLED display.
//
// Test sequence:
//   1. LEDs (all on → all off)
//   2. Buzzer (300 ms beep)
//   3. Servo  (0° → 180° → 90°)
//   4. Motor  (fwd 800 ms → rev 800 ms → stop)
//   5. Encoder (read & display current count)
//   6. I²C sensors (re-init MPU6050 + QMC5883L)
//   7. Button press (5 s timeout)
//   8. Live IMU display (20 frames)
// =============================================================
#pragma once

void runFullTest();
void turnAngleTest(float turnAngleDeg = 90.0f, int speed = 55);



