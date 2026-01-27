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
#define PIN_DISP_RST  9
#define PIN_FILAMENT  14
#define PIN_LDR       10

// WiFi defaults (AP mode for zero-dependency bring-up)
#define WIFI_AP_SSID      "ESP32-Lathe"
#define WIFI_AP_PASSWORD  "lathe1234"   // set to "" for open AP