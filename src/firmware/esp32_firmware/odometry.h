// =============================================================
// odometry.h  –  Position, heading, and distance tracking.
// =============================================================
#pragma once

#include <Arduino.h>

// ── Initialise tracking state ─────────────────────────────
void initOdometry();

// ── Update calculations (call in main loop) ───────────────
void updateOdometry();
