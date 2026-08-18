// =============================================================
// encoder.cpp  –  Quadrature encoder implementation.
// =============================================================
#include "encoder.h"
#include <Arduino.h>
#include <driver/pcnt.h>
#include "config.h"

// ── Configure PCNT for 2x quadrature decoding ─────────────
void initEncoder() {
  // Input-only pins: no internal pull-up available on GPIO 34/35.
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  pcnt_config_t cfg = {
    .pulse_gpio_num = ENC_A_PIN,
    .ctrl_gpio_num  = ENC_B_PIN,
    .lctrl_mode     = PCNT_MODE_REVERSE,  // B LOW  → normal counting direction
    .hctrl_mode     = PCNT_MODE_KEEP,     // B HIGH → reverse counting direction
    .pos_mode       = PCNT_COUNT_INC,     // rising  edge of A → increment
    .neg_mode       = PCNT_COUNT_DEC,     // falling edge of A → decrement
    .counter_h_lim  =  32767,
    .counter_l_lim  = -32768,
    .unit           = PCNT_UNIT_0,
    .channel        = PCNT_CHANNEL_0
  };

  pcnt_unit_config(&cfg);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

// ── Read current encoder count ────────────────────────────
int32_t getEncoderCount() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  return (int32_t)count;
}

// ── Reset encoder count to zero ───────────────────────────
void resetEncoder() {
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}
