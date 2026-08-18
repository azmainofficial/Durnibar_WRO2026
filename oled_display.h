// =============================================================
// oled_display.h  –  OLED status screen.
//
// Provides:
//   updateOLED()  – refresh the 4-line status display
//
// Layout (128×64, System5x7 font, 1× scale):
//   Line 1: Servo angle  |  Motor speed
//   Line 2: Encoder count  |  Heading
//   Line 3: Roll  |  Pitch
//   Line 4: Calibration status  |  Active buttons
// =============================================================
#pragma once

void updateOLED();

