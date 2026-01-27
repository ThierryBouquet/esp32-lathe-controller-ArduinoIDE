#pragma once
#include <Arduino.h>

typedef struct { uint16_t min, max; float amb_min, amb_max, gamma; } bright_map_t;

typedef struct {
  // live state
  float rpm_meas, rpm_set, css_v, dia_mm;
  bool  mode_css, spindle_on;
  // brightness
  bool  bright_auto;
  uint16_t bright_manual; // 0..500 (GP1294 safe cap)
  bright_map_t bmap;      // not used in Step-1
} lathe_state_t;

typedef struct {
  float kp, ki, kd, out_min, out_max;
  uint16_t rpm_min, rpm_max, css_min, css_max; float belt_ratio;
} lathe_config_t;

extern lathe_state_t g_state;