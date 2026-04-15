#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can_task.hpp"
#include "shifter_task.hpp"

static const char *TAG = "APP_MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting application tasks");

    xTaskCreate(can_task, "can_task", 4096, nullptr, 1, nullptr);
    xTaskCreate(shifter_task, "shifter_task", 4096, nullptr, 2, nullptr);

     vTaskDelete(NULL);
}
