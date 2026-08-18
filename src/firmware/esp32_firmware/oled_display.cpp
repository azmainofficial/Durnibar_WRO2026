// =============================================================
// oled_display.cpp  –  OLED status screen implementation.
// =============================================================
#include "oled_display.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"

void updateOLED() {
  oled.home();
  oled.set1X();

  // Row 1 – servo angle & motor speed command
  oled.print("Srv:"); oled.print(currentServoAngle); oled.print((char)247);
  oled.print(" Cmd:"); oled.print(motorSpeed);
  oled.println();

  // Row 2 – total distance & speed (mm/s)
  oled.print("Dst:"); oled.print((int)odomDist); oled.print(" S:"); oled.print((int)odomSpeed);
  oled.println();

  // Row 3 – 2D coordinates (X, Y) in mm
  oled.print("X:"); oled.print((int)odomX); oled.print(" Y:"); oled.print((int)odomY);
  oled.println();

  // Row 4 – fused yaw heading & calibration status
  oled.print("Yaw:"); oled.print((int)imuYaw); oled.print((char)247);
  oled.print(" Cal:"); oled.print(calData.isCalibrated ? "OK" : "NO");
  oled.println();

}
