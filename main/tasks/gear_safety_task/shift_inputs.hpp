#pragma once

#include "esp_attr.h"

void setup_shift_inputs();

void IRAM_ATTR shift_up_isr(void *arg);
void IRAM_ATTR shift_down_isr(void *arg);
void IRAM_ATTR shift_neutral_isr(void *arg);
