#include "hal_display_gp1294.h"
#include "config_defaults.h"
#include <U8x8lib.h>

// U8X8 text-mode API for GP1294, software SPI
// Signature: (clock, data, cs, dc, reset)
// GP1294AI does not use DC -> U8X8_PIN_NONE
static U8X8_GP1294AI_256X48_4W_SW_SPI u8x8(/* clock=*/ PIN_DISP_CLK,
                                           /* data=*/  PIN_DISP_MOSI,
                                           /* cs=*/    PIN_DISP_CS,
                                           /* dc=*/    U8X8_PIN_NONE,
                                           /* reset=*/ PIN_DISP_RST);

void hal_display_init() {
  pinMode(PIN_FILAMENT, OUTPUT);
  digitalWrite(PIN_FILAMENT, HIGH); // enable filament

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setContrast(200);            // safe default
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clearDisplay();
  u8x8.drawString(0, 1, "ESP32 Lathe");
  u8x8.drawString(0, 3, "Display OK");
}

void hal_display_draw2(const char* l1, const char* l2) {
  u8x8.clearDisplay();
  u8x8.drawString(0, 1, l1 ? l1 : "");
  u8x8.drawString(0, 3, l2 ? l2 : "");
}

void hal_display_set_brightness(uint16_t level) {
  // TODO: map level (0..500) into GP1294 DIM levels if exposed via command.
  (void)level;
}