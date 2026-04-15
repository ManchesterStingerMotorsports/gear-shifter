#include "gear_safety_task.hpp"
#include "shift_inputs.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG_GEAR_SAFETY = "GEAR_SAFETY_TASK";

enum gear_t {GEAR_N, GEAR_1, GEAR_2, GEAR_3, GEAR_4, GEAR_5};

void gear_safety_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG_GEAR_SAFETY, "Gear safety task started");
    setup_shift_inputs();

    gear_t gear_count = GEAR_N;
    

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}