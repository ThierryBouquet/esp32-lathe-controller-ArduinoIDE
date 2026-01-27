#pragma once
#include <Arduino.h>

void hal_display_init();
void hal_display_draw2(const char* l1, const char* l2);
void hal_display_set_brightness(uint16_t level); // stub in Step-1