// =============================================================
// pi_comm.h  –  Raspberry Pi 5 Communication & Telemetry.
// =============================================================
#pragma once

#include <Arduino.h>

// ── Initialize Pi Communication ───────────────────────────
void initPiComm();

// ── Process incoming commands & send 20 Hz telemetry stream ──
void updatePiComm();
