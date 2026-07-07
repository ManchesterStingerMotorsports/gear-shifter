#include "can_task.hpp"

#include <cstdint>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_SYS = "CAN_TEST_SYSTEM";

static constexpr BaseType_t CAN_TEST_CORE = 0;
static constexpr UBaseType_t CAN_TEST_TASK_PRIORITY = tskIDLE_PRIORITY + 1;
static constexpr uint32_t CAN_TEST_TASK_STACK_WORDS = 4096;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_SYS, "Creating CAN test task");

    const BaseType_t ok = xTaskCreatePinnedToCore(can_task,
                                                  "can_test_task",
                                                  CAN_TEST_TASK_STACK_WORDS,
                                                  nullptr,
                                                  CAN_TEST_TASK_PRIORITY,
                                                  nullptr,
                                                  CAN_TEST_CORE);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG_SYS, "CAN test task creation failed");
        taskDISABLE_INTERRUPTS();
        for (;;)
        {
        }
    }

    ESP_LOGI(TAG_SYS, "CAN test task created");
    vTaskDelete(nullptr);
}
