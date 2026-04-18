#include "can_task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "MSM_CAN.hpp"
#include "gear_state.hpp"

static const char *TAG = "CAN_TASK";

void can_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "CAN task started");

    MSM_CAN::set_hardware_filters(0x360, 0x470); //allow a range of ids from 360-470

    esp_err_t err = MSM_CAN::init(GPIO_NUM_2, GPIO_NUM_1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "CAN init failed: %s", esp_err_to_name(err));
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    err = MSM_CAN::subscribe(0x360); //rpm packet for future addition of downshift protection
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "CAN subscribe failed for 0x360: %s", esp_err_to_name(err));
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    err = MSM_CAN::subscribe(0x470); //gear packet with the ecu's calculated gear
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "CAN subscribe failed for 0x470: %s", esp_err_to_name(err));
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    ESP_LOGI(TAG, "CAN initialised and subscribed to 0x360, 0x470");

    uint8_t gear_data[8];
    uint32_t gear_timestamp_ms = 0;

    for (;;)
    {
        if (MSM_CAN::get(0x470, gear_data, &gear_timestamp_ms) == ESP_OK)
        {
            const int8_t raw_gear = MSM_CAN::unpack_i8(gear_data, 7);

            if (raw_gear >= 0 && raw_gear <= 5)
            {
                const gear_t ecu_gear = static_cast<gear_t>(raw_gear);
                update_ecu_gear_snapshot(ecu_gear, gear_timestamp_ms);
                ESP_LOGD(TAG, "ECU gear update: gear=%d timestamp_ms=%lu",
                         static_cast<int>(ecu_gear),
                         static_cast<unsigned long>(gear_timestamp_ms));
            }
            else
            {
                ESP_LOGW(TAG, "Ignoring invalid ECU gear value: %d", static_cast<int>(raw_gear));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); //task runs every 50ms
    }
}
