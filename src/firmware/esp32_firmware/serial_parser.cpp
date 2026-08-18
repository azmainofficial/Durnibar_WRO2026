// =============================================================
// serial_parser.cpp  –  Serial integer parser implementation.
// =============================================================
#include "serial_parser.h"
#include <Arduino.h>

int readSerialAngle() {
  static String input = "";

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        int val = input.toInt();
        input = "";
        return val;
      }
    } else if ((c >= '0' && c <= '9') || c == '-') {
      // Explicit parentheses around the digit check to avoid
      // operator-precedence ambiguity with the || c == '-' clause.
      input += c;
    } else {
      input = "";  // discard partial input on unexpected character
    }
  }

  return -1;  // no complete number received yet
}
