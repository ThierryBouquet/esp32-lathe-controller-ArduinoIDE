#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── Runtime-tunable configuration ───────────────────────────────

typedef struct {
  float    kp, ki, kd;
  float    out_min, out_max;       // PWM output limits
  uint16_t rpm_min, rpm_max;       // RPM range for pot mapping & CSS clamp
  uint16_t css_min, css_max;       // CSS range for pot mapping
  float    belt_ratio;
} lathe_config_t;

extern lathe_config_t  g_config;
extern SemaphoreHandle_t g_config_mutex;

// ── Live state (written by tasks, read by display/web) ──────────

typedef struct lathe_state_t {
  // Measured values
  float    rpm_meas;           // Actual spindle RPM
  float    css_v;              // Calculated CSS (m/min)
  float    dia_mm;             // Diameter from DRO X

  // Switches & pot (from sensor_io_task)
  bool     mode_css;           // true = CSS mode, false = RPM mode
  bool     spindle_motor_on;   // Physical spindle switch state
  uint16_t speed_pot_raw;      // Raw ADC (0-4095)
  float    speed_percent;      // Pot as 0-100%

  // Setpoints (derived from pot + config ranges)
  uint16_t rpm_setpoint;
  uint16_t css_setpoint;

  // PID output
  uint8_t  pwm_duty;           // Current PWM duty cycle (0-255)

  // Display brightness
  bool     bright_auto;
  uint16_t bright_manual;      // 0..500

  // DRO
  float    dro_x_pos;
  float    dro_z_pos;
  bool     dro_connected;
  bool     dro_lost_in_css;      // DRO disconnected while in CSS mode

  // Auto-tune
  bool     autotune_active;
  bool     autotune_done;
  float    autotune_kp, autotune_ki, autotune_kd;

  // System
  bool     wifi_connected;
  char     time_text[16];
} lathe_state_t;

extern lathe_state_t     g_state;
extern SemaphoreHandle_t g_state_mutex;