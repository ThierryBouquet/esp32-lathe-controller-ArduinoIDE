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
#define PIN_LDR       9

// Spindle motor state input
#define PIN_SPINDLE_MOTOR  3

// RS485 / Modbus RTU for DRO (Hardware Serial 2)
#define PIN_RS485_TX  17
#define PIN_RS485_RX  16
#define PIN_RS485_DE  15   // Driver Enable (DE/RE tied together)
#define DRO_MODBUS_ADDR  1
#define DRO_BAUD_RATE    9600

// WiFi defaults (Station mode - connects to your network)
#define WIFI_SSID      "Zyxel_338D"     // Change to your WiFi network name
#define WIFI_PASSWORD  "p3pabjnf8m4h7f8e" // Change to your WiFi password