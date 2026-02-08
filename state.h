#pragma once
#include <Arduino.h>

typedef struct { uint16_t min, max; float amb_min, amb_max, gamma; } bright_map_t;

typedef struct {
  // live state
  float rpm_meas, rpm_set, css_v, dia_mm;
  bool  mode_css, spindle_on;
  bool  spindle_motor_on;  // Physical spindle motor state from GPIO
  // brightness
  bool  bright_auto;
  uint16_t bright_manual; // 0..500 (GP1294 safe cap)
  bright_map_t bmap;      // not used in Step-1
  
  // Testing and display variables
  uint16_t test_rpm;      // 0-6000 (for testing)
  uint16_t test_css;      // 0-500 (for testing)
  uint16_t rpm_setpoint;  // 0-6000 (RPM setpoint)
  uint16_t css_setpoint;  // 0-500 (CSS setpoint)
  float test_x_coord;     // -999.99 to 999.99 (for testing)
  float test_z_coord;     // -99.99 to 99.99 (for testing)
  
  // DRO actual positions (from Modbus)
  float dro_x_pos;        // X axis position from DRO
  float dro_z_pos;        // Z axis position from DRO
  bool dro_connected;     // DRO communication status
  bool wifi_connected;    // WiFi connection status
  char time_text[16];     // Current time HH:MM:SS
} lathe_state_t;

typedef struct {
  float kp, ki, kd, out_min, out_max;
  uint16_t rpm_min, rpm_max, css_min, css_max; float belt_ratio;
} lathe_config_t;

extern lathe_state_t g_state;