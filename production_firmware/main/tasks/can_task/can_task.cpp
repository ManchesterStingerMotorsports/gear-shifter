#include "can_task.hpp"

#include "can_status.hpp"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_CAN_TASK = "CAN_TASK";

void can_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_CAN_TASK, "CAN task started");
    set_can_task_output({false, 0, false});

    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
