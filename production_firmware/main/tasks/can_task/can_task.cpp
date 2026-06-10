#include "can_task.hpp"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

static const char *TAG_CAN_TASK = "CAN_TASK";

static portMUX_TYPE can_task_output_mux = portMUX_INITIALIZER_UNLOCKED;

static CanTaskOutput can_task_output = {false, 0, false};

CanTaskOutput get_can_task_output()
{
    CanTaskOutput output;

    portENTER_CRITICAL(&can_task_output_mux);
    output = can_task_output;
    portEXIT_CRITICAL(&can_task_output_mux);

    return output;
}

void set_can_task_output(const CanTaskOutput &output)
{
    portENTER_CRITICAL(&can_task_output_mux);
    can_task_output = output;
    portEXIT_CRITICAL(&can_task_output_mux);
}

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
