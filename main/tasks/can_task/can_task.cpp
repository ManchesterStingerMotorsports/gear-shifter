#include "can_task.hpp"

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "MSM_CAN.hpp"

#ifndef APP_CAN_RX_GPIO_NUM
#define APP_CAN_RX_GPIO_NUM 4
#endif

#ifndef APP_CAN_TX_GPIO_NUM
#define APP_CAN_TX_GPIO_NUM 5
#endif

constexpr uint16_t CAN_STATUS_ID = 0x120;
constexpr uint32_t CAN_TASK_PERIOD_MS = 100;
constexpr gpio_num_t CAN_RX_GPIO = static_cast<gpio_num_t>(APP_CAN_RX_GPIO_NUM);
constexpr gpio_num_t CAN_TX_GPIO = static_cast<gpio_num_t>(APP_CAN_TX_GPIO_NUM);

static const char *TAG = "CAN_TASK";

void can_callback(uint16_t id, const uint8_t data[8], uint32_t timestamp_ms)
{
    ESP_LOGD(TAG,
             "RX id=0x%03" PRIx16 " ts=%" PRIu32 " data0=0x%02x",
             static_cast<uint16_t>(id),
             timestamp_ms,
             static_cast<unsigned int>(data[0]));
}

void can_task(void *arg)
{
    (void)arg;

    MSM_CAN::set_hardware_filters();

    esp_err_t err = MSM_CAN::init(CAN_RX_GPIO, CAN_TX_GPIO);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MSM_CAN init failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "MSM_CAN started on RX GPIO %d / TX GPIO %d",
                 static_cast<int>(CAN_RX_GPIO),
                 static_cast<int>(CAN_TX_GPIO));

        err = MSM_CAN::subscribe(CAN_STATUS_ID, can_callback);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "CAN subscribe failed: %s", esp_err_to_name(err));
        }
    }

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_PERIOD_MS));
    }
}
