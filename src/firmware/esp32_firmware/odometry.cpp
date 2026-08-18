// =============================================================
// odometry.cpp  –  Odometry tracking implementation.
// =============================================================
#include "odometry.h"
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "globals.h"
#include "encoder.h"


// Static local variables to track changes between loop updates
static int32_t       lastTicks       = 0;
static unsigned long lastSpeedTime   = 0;
static float         lastOdomDist    = 0.0f;
static float         kinematicYawRad = 0.0f;  // Fallback heading in radians

// ── Initialise tracking state ─────────────────────────────
void initOdometry() {
  lastTicks       = getEncoderCount();
  lastSpeedTime   = millis();
  lastOdomDist    = 0.0f;
  kinematicYawRad = imuYaw * DEG_TO_RAD; // Start kinematic yaw synced with initial yaw
  
  odomX     = 0.0f;
  odomY     = 0.0f;
  odomDist  = 0.0f;
  odomSpeed = 0.0f;
}

// ── Update calculations (call in main loop) ───────────────
void updateOdometry() {
  int32_t currentTicks = getEncoderCount();
  int32_t deltaTicks   = currentTicks - lastTicks;

  // 1. Calculate linear distance traveled in this step
  float deltaDist = 0.0f;
  if (deltaTicks != 0) {
#if defined(ENCODER_INVERT_DIR) && (ENCODER_INVERT_DIR == true)
    deltaTicks = -deltaTicks;
#endif
    deltaDist = ( (float)deltaTicks / EFFECTIVE_ENCODER_CPR ) * WHEEL_CIRCUMFERENCE_MM;
    odomDist += abs(deltaDist); // Cumulative path distance (always positive)
    lastTicks = currentTicks;
  }


  // 2. Track heading (Yaw)
  float headingRad = 0.0f;
  if (sensorsOK) {
    // IMU is functional: use the fused imuYaw heading
    headingRad = imuYaw * DEG_TO_RAD;
    // Keep fallback kinematic yaw synced with IMU yaw to be ready if sensor fails later
    kinematicYawRad = headingRad;
  } else {
    // FALLBACK: IMU failed, use steering kinematic bicycle model
    // Convert servo deflection to wheel steer angle in radians
    float steerServoDelta = (float)currentServoAngle - STEER_SERVO_CENTER;
    
    // Scale servo offset (-90..90) to wheel angle (-STEER_MAX_ANGLE_DEG..STEER_MAX_ANGLE_DEG)
    float steerAngleRad = (steerServoDelta / 90.0f) * STEER_MAX_ANGLE_DEG * DEG_TO_RAD;

    // Calculate heading change: deltaYaw = (deltaDist / L) * tan(steering_angle)
    float deltaYaw = (deltaDist / WHEELBASE_MM) * tan(steerAngleRad);
    kinematicYawRad += deltaYaw;

    // Keep angle bound to [0, 2*PI)
    while (kinematicYawRad < 0.0f)     kinematicYawRad += TWO_PI;
    while (kinematicYawRad >= TWO_PI)  kinematicYawRad -= TWO_PI;

    // Output heading to global state
    imuYaw = kinematicYawRad * RAD_TO_DEG;
    headingRad = kinematicYawRad;
  }

  // 3. Update 2D Position coordinates
  if (deltaTicks != 0) {
    // Integrate position: dX = dDist * cos(heading), dY = dDist * sin(heading)
    // Note: deltaDist carries the direction (positive/negative)
    odomX += deltaDist * cos(headingRad);
    odomY += deltaDist * sin(headingRad);
  }

  // 4. Calculate Speed (mm/s) updated at 10 Hz (every 100 ms)
  unsigned long now = millis();
  unsigned long dt_ms = now - lastSpeedTime;
  if (dt_ms >= 100) {
    float distDelta = odomDist - lastOdomDist;
    float rawSpeed = (distDelta / (float)dt_ms) * 1000.0f; // mm/s

    // Low-pass filter for smooth speed readings
    odomSpeed = (0.7f * odomSpeed) + (0.3f * rawSpeed);

    lastOdomDist = odomDist;
    lastSpeedTime = now;
  }
}
