#include "hal_display_gp1294.h"
#include "config_defaults.h"
#include <U8x8lib.h>
#include <string.h>

// U8X8 text-mode API for GP1294, software SPI
// Signature: (clock, data, cs, dc, reset)
// GP1294AI does not use DC -> U8X8_PIN_NONE
static U8X8_GP1294AI_256X48_4W_SW_SPI u8x8(/* clock=*/ PIN_DISP_CLK,
                                           /* data=*/  PIN_DISP_MOSI,
                                           /* cs=*/    PIN_DISP_CS,
                                           /* dc=*/    U8X8_PIN_NONE,
                                           /* reset=*/ PIN_DISP_RST);

// Track previous line contents to avoid full-screen clears
static char last1[33] = "";
static char last2[33] = "";

// Track last applied contrast to avoid redundant I/O
static uint8_t last_contrast = 0xFF; // invalid to force first set

// 32 spaces for line wipe
static const char* SPACES_32 = "                                ";

static uint8_t clamp_u8(int v) { return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

void hal_display_init() {
  pinMode(PIN_FILAMENT, OUTPUT);
  digitalWrite(PIN_FILAMENT, HIGH); // enable filament

  u8x8.begin();
  u8x8.setPowerSave(0);
  // Default contrast (will be overridden by hal_display_set_brightness)
  u8x8.setContrast(140);
  last_contrast = 140;

  u8x8.setFont(u8x8_font_chroma48medium8_r);

  // One-time clear on init only
  u8x8.clearDisplay();
  u8x8.drawString(0, 1, "ESP32 Lathe");
  u8x8.drawString(0, 3, "Display OK");

  // Initialize last-line caches to force first overwrite on next draw
  last1[0] = '\0';
  last2[0] = '\0';
}

void hal_display_draw2(const char* l1, const char* l2) {
  const char* s1 = l1 ? l1 : "";
  const char* s2 = l2 ? l2 : "";

  // Update line 1 only if changed
  if (strcmp(s1, last1) != 0) {
    u8x8.drawString(0, 1, SPACES_32);   // clear line by overwriting with spaces
    u8x8.drawString(0, 1, s1);
    strncpy(last1, s1, sizeof(last1) - 1);
    last1[sizeof(last1) - 1] = 0;
  }

  // Update line 3 only if changed
  if (strcmp(s2, last2) != 0) {
    u8x8.drawString(0, 3, SPACES_32);   // clear line by overwriting with spaces
    u8x8.drawString(0, 3, s2);
    strncpy(last2, s2, sizeof(last2) - 1);
    last2[sizeof(last2) - 1] = 0;
  }
}

void hal_display_set_brightness(uint16_t level) {
  // Map 0..500 (UI range) -> ~10..255 contrast (avoid total black at very low)
  // Simple linear map; tune later with gamma if needed
  int mapped = (int)((level / 500.0f) * 255.0f + 0.5f);
  if (mapped < 10) mapped = 10; // small floor so text remains visible
  uint8_t c = clamp_u8(mapped);

  if (c != last_contrast) {
    u8x8.setContrast(c);
    last_contrast = c;
  }
}
