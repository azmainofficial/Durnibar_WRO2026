// =============================================================
// encoder.h  –  Quadrature encoder via ESP32 PCNT hardware.
//
// Uses PCNT_UNIT_0 / PCNT_CHANNEL_0 for reliable 2x decoding.
// GPIO 34 & 35 have NO internal pull-ups; fit external 10 kΩ
// pull-up resistors to 3.3 V.
//
// Provides:
//   initEncoder()       – configure PCNT & start counting
//   getEncoderCount()   – return current 16-bit signed count
//   resetEncoder()      – zero the counter
// =============================================================
#pragma once

#include <Arduino.h>

// ── Configure PCNT for 2x quadrature decoding ─────────────
void initEncoder();

// ── Read current encoder count ────────────────────────────
int32_t getEncoderCount();

// ── Reset encoder count to zero ───────────────────────────
void resetEncoder();

