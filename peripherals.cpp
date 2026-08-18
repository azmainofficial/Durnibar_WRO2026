// =============================================================
// peripherals.cpp  –  LEDs, buzzer, and buttons implementation.
// =============================================================
#include "peripherals.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"

// ── Set a single LED explicitly ───────────────────────────
void setLED(int pin, bool &state, bool val) {
  state = val;
  digitalWrite(pin, state ? HIGH : LOW);
}

// ── Toggle a single LED and print status to Serial ────────
void toggleLED(int pin, bool &state) {
  setLED(pin, state, !state);
  Serial.print("LED ");
  Serial.print(pin);
  Serial.println(state ? " ON" : " OFF");
}

// ── Start a non-blocking buzzer beep ──────────────────────
// Records the time the buzzer should turn off.
void beepBuzzer(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOffTime = millis() + duration;
  Serial.println("Buzzer beep");
}

// ── Non-blocking buzzer state machine ─────────────────────
// Must be called every loop() to turn the buzzer off on time.
void updateBuzzer() {
  if (buzzerOffTime > 0 && millis() >= buzzerOffTime) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOffTime = 0;
  }
}

#include "hardware_test.h"

// ── Button action callbacks ───────────────────────────────
// BUTTON 1 → RUN BOT (Trigger Open Challenge on Pi)
void handleButton1() {
  beepBuzzer(80);
  btn1Triggered = true;
}

// BUTTON 2 → STOP BOT (Trigger Emergency Stop on Pi)
void handleButton2() {
  beepBuzzer(160);
  btn2Triggered = true;
}


