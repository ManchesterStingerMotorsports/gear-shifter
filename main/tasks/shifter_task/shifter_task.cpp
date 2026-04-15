#include "shifter_task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

constexpr uint32_t SHIFTER_TASK_PERIOD_MS = 20;

static const char *TAG = "SHIFTER_TASK";

void shifter_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Shifter task started");

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(SHIFTER_TASK_PERIOD_MS));
    }
}
