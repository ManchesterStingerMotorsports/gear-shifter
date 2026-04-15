#include "shifter_task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "SHIFTER_TASK";

void shifter_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Shifter task started");

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
