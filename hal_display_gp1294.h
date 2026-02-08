#pragma once
#include <Arduino.h>

void hal_display_init();
void hal_display_draw_lathe();  // New graphical display function
void hal_display_set_brightness(uint16_t level);