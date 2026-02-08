#include "hal_display_gp1294.h"
#include "config_defaults.h"
#include "state.h"
#include <U8g2lib.h>

// U8g2 full buffer graphics mode for GP1294, software SPI
// Signature: (clock, data, cs, dc, reset)
// GP1294AI does not use DC -> U8X8_PIN_NONE
U8G2_GP1294AI_256X48_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ PIN_DISP_CLK,
                                                /* data=*/  PIN_DISP_MOSI,
                                                /* cs=*/    PIN_DISP_CS,
                                                /* dc=*/    U8X8_PIN_NONE,
                                                /* reset=*/ PIN_DISP_RST);

// Static images (bitmaps)
static const unsigned char image_clock_bits[] = {
  0xe0,0x03,0x18,0x0c,0x94,0x14,0x82,0x20,0x86,0x30,0x81,0x40,
  0x81,0x40,0x87,0x70,0x01,0x41,0x01,0x42,0x06,0x34,0x02,0x20,
  0x94,0x14,0x98,0x0c,0xe0,0x03,0x00,0x00
};

static const unsigned char image_wifi_full_bits[] = {
  0x80,0x0f,0x00,0xe0,0x3f,0x00,0x78,0xf0,0x00,0x9c,0xcf,0x01,
  0xee,0xbf,0x03,0xf7,0x78,0x07,0x3a,0xe7,0x02,0xdc,0xdf,0x01,
  0xe8,0xb8,0x00,0x70,0x77,0x00,0xa0,0x2f,0x00,0xc0,0x1d,0x00,
  0x80,0x0a,0x00,0x00,0x07,0x00,0x00,0x02,0x00,0x00,0x00,0x00
};

static const unsigned char image_wifi_not_connected_bits[] = {
  0x84,0x0f,0x00,0x68,0x30,0x00,0x10,0xc0,0x00,0xa4,0x0f,0x01,
  0x42,0x30,0x02,0x91,0x40,0x04,0x08,0x85,0x00,0xc4,0x1a,0x01,
  0x20,0x24,0x00,0x10,0x4a,0x00,0x80,0x15,0x00,0x40,0x20,0x00,
  0x00,0x42,0x00,0x00,0x85,0x00,0x00,0x02,0x01,0x00,0x00,0x00
};

// Track last applied contrast to avoid redundant I/O
static uint8_t last_contrast = 0xFF;

static uint8_t clamp_u8(int v) { 
  return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); 
}

void hal_display_init() {
  pinMode(PIN_FILAMENT, OUTPUT);
  digitalWrite(PIN_FILAMENT, HIGH); // enable filament

  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setContrast(140);
  last_contrast = 140;
}

extern lathe_state_t g_state;

void hal_display_draw_lathe() {
  // Format text values
  char CSS_value_text[8];
  char RPM_value_text[8];
  char x_reading_text[12];
  char z_reading_text[12];
  
  snprintf(CSS_value_text, sizeof(CSS_value_text), "%d", g_state.test_css);
  snprintf(RPM_value_text, sizeof(RPM_value_text), "%d", g_state.test_rpm);
  snprintf(x_reading_text, sizeof(x_reading_text), "%+07.2f", g_state.test_x_coord);
  snprintf(z_reading_text, sizeof(z_reading_text), "%+06.2f", g_state.test_z_coord);
  
  // Calculate indicator bar widths (0-130 pixels)
  int RPM_indicator_frame_w = map(constrain(g_state.test_rpm, 0, 6000), 0, 6000, 0, 130);
  int CSS_indicator_frame_w = map(constrain(g_state.test_css, 0, 500), 0, 500, 0, 130);
  
  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  
  // === STATIC FRAMES ===
  u8g2.setDrawColor(1);
  
  // Time frame (top left)
  u8g2.drawRFrame(0, 0, 79, 23, 3);
  
  // WiFi frame (bottom left)
  u8g2.drawRFrame(0, 25, 25, 23, 3);
  
  // Coordinate frame (bottom middle)
  u8g2.drawRFrame(27, 25, 52, 23, 3);
  
  // RPM value frame (top right)
  u8g2.drawRFrame(124, 0, 132, 23, 3);
  
  // CSS value frame (bottom right)
  u8g2.drawRFrame(124, 25, 132, 23, 3);
  
  // === MODE-DEPENDENT FRAMES AND INDICATORS ===
  if (!g_state.mode_css) {
    // RPM MODE ACTIVE
    // RPM mode indicator - filled rounded box
    u8g2.setDrawColor(1);
    u8g2.drawRBox(81, 0, 42, 23, 3);
    
    // CSS mode indicator - rounded outline only
    u8g2.drawRFrame(81, 25, 42, 23, 3);
    
    // RPM indicator bar - filled
    u8g2.drawBox(125, 1, RPM_indicator_frame_w, 21);
    
    // CSS indicator bar - outline only
    u8g2.drawFrame(125, 26, CSS_indicator_frame_w, 21);
  } else {
    // CSS MODE ACTIVE
    // RPM mode indicator - rounded outline only
    u8g2.setDrawColor(1);
    u8g2.drawRFrame(81, 0, 42, 23, 3);
    
    // CSS mode indicator - filled rounded box
    u8g2.drawRBox(81, 25, 42, 23, 3);
    
    // RPM indicator bar - outline only
    u8g2.drawFrame(125, 1, RPM_indicator_frame_w, 21);
    
    // CSS indicator bar - filled
    u8g2.drawBox(125, 26, CSS_indicator_frame_w, 21);
  }
  
  // === MODE LABELS ===
  u8g2.setFont(u8g2_font_profont22_tr);
  u8g2.setDrawColor(2);
  u8g2.drawStr(84, 18, "RPM");
  u8g2.drawStr(84, 43, "CSS");
  
  // === ICONS ===
  u8g2.setDrawColor(1);
  // Clock icon
  u8g2.drawXBM(2, 3, 15, 16, image_clock_bits);
  
  // WiFi icon (conditional)
  if (g_state.wifi_connected) {
    u8g2.drawXBM(3, 29, 19, 16, image_wifi_full_bits);
  } else {
    u8g2.drawXBM(3, 29, 19, 16, image_wifi_not_connected_bits);
  }
  
  // === TEXT VALUES ===
  // Time
  u8g2.setFont(u8g2_font_profont15_tr);
  u8g2.drawStr(20, 15, g_state.time_text);
  
  // Coordinate labels and values
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(30, 35, "X");
  u8g2.drawStr(30, 45, "Z");
  u8g2.drawStr(35, 35, x_reading_text);
  u8g2.drawStr(41, 45, z_reading_text);
  
  // RPM and CSS values
  u8g2.setFont(u8g2_font_profont22_tr);
  u8g2.setDrawColor(2);
  u8g2.drawStr(165, 18, RPM_value_text);
  u8g2.drawStr(165, 43, CSS_value_text);
  
  u8g2.sendBuffer();
}

void hal_display_set_brightness(uint16_t level) {
  // Map 0..500 (UI range) -> ~10..255 contrast (avoid total black at very low)
  int mapped = (int)((level / 500.0f) * 255.0f + 0.5f);
  if (mapped < 10) mapped = 10; // small floor so text remains visible
  uint8_t c = clamp_u8(mapped);

  if (c != last_contrast) {
    u8g2.setContrast(c);
    last_contrast = c;
  }
}
