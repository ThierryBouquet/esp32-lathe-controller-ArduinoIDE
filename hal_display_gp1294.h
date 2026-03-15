#pragma once
#include <Arduino.h>
#include "state.h"

void hal_display_init();
void hal_display_draw_lathe(const lathe_state_t *s);
void hal_display_set_brightness(uint16_t level);