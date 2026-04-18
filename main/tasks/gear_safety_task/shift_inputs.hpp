#pragma once

#include <stdint.h>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Shift request values latched by the GPIO ISRs.
enum ShiftRequest
{
    SHIFT_REQUEST_NONE = 0,
    SHIFT_REQUEST_UP,
    SHIFT_REQUEST_DOWN,
    SHIFT_REQUEST_NEUTRAL,
};

// Configure the shift input GPIOs and register their ISRs.
void setup_shift_inputs(TaskHandle_t shift_task_handle);

// Take the pending request and clear the shared latch.
ShiftRequest consume_pending_shift_request();

// Clear stale input state and re-enable the shift GPIO interrupts.
void clear_and_enable_shift_inputs();

void IRAM_ATTR shift_up_isr(void *arg);
void IRAM_ATTR shift_down_isr(void *arg);
void IRAM_ATTR shift_neutral_isr(void *arg);
