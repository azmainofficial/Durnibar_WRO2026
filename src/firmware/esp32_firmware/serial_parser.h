// =============================================================
// serial_parser.h  –  Read an integer value from the Serial port.
//
// Provides:
//   readSerialAngle()  – accumulates digits until newline, then
//                        returns the parsed integer.  Returns -1
//                        if no complete number is available yet.
//
// Accepts digits 0-9 and a leading '-' for negative numbers.
// Any other character resets the accumulator.
// =============================================================
#pragma once

int readSerialAngle();

