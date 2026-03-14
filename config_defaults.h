#pragma once
// Defaults for quick bring-up
#define DEF_KP 0.5f
#define DEF_KI 0.1f
#define DEF_KD 0.0f
#define DEF_OUT_MIN 0.0f
#define DEF_OUT_MAX 1.0f
#define DEF_RPM_MIN 100
#define DEF_RPM_MAX 2000
#define DEF_CSS_MIN 10
#define DEF_CSS_MAX 300
#define DEF_BELT_RATIO (15.0f/32.0f)

// Display pins (match your wiring)
#define PIN_DISP_MOSI 13
#define PIN_DISP_CLK  12
#define PIN_DISP_CS   11
#define PIN_DISP_RST  10
#define PIN_FILAMENT  14
// PIN_LDR removed - pin 9 now used for speed potentiometer

// Spindle motor control
#define PIN_SPINDLE_PWM    8   // PWM output to motor controller
#define PIN_SPEED_POT      9   // ADC input for speed adjustment potentiometer
#define PIN_MODE_SWITCH    3  // Digital input for mode switch (RPM/CSS)
#define PIN_SPINDLE_SWITCH 36   // Digital input for spindle motor on/off switch
#define PIN_SPINDLE_RPM    46  // Input to read spindle RPM pulses

// PWM Configuration
#define PWM_FREQUENCY   5000   // 5 kHz PWM frequency
#define PWM_RESOLUTION     8   // 8-bit resolution (0-255)

// RS485 / Modbus RTU for DRO (Hardware Serial 2)
#define PIN_RS485_TX  17
#define PIN_RS485_RX  16
#define PIN_RS485_DE  15   // Driver Enable (DE/RE tied together)
#define DRO_MODBUS_ADDR  1
#define DRO_BAUD_RATE    9600

// WiFi defaults (Station mode - connects to your network)
#define WIFI_SSID      "Zyxel_338D"     // Change to your WiFi network name
#define WIFI_PASSWORD  "p3pabjnf8m4h7f8e" // Change to your WiFi password