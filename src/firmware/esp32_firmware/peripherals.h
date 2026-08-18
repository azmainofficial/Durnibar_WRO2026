// =============================================================
// peripherals.h  –  LEDs, buzzer (non-blocking), and buttons.
//
// Provides:
//   toggleLED(pin, state)  – flip one LED and log to Serial
//   beepBuzzer(duration)   – start a non-blocking beep
//   updateBuzzer()         – call every loop() to end the beep
//   handleButton1()        – toggle Red LED  (bound to BUTTON1_PIN)
//   handleButton2()        – toggle Green LED (bound to BUTTON2_PIN)
// =============================================================
#pragma once

#include <Arduino.h>

// ── Set or toggle a single LED and print status to Serial ─
void toggleLED(int pin, bool &state);
void setLED(int pin, bool &state, bool val);

// ── Start a non-blocking buzzer beep ──────────────────────
void beepBuzzer(int duration = 200);

// ── Non-blocking buzzer state machine ─────────────────────
void updateBuzzer();

// ── Button action callbacks ───────────────────────────────
void handleButton1();
void handleButton2();

