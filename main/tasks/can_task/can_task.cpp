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
    MSM_CAN::init(GPIO_NUM_2, GPIO_NUM_1);

    MSM_CAN::subscribe(0x360); //rpm packet for future addition of downshift protection
    MSM_CAN::subscribe(0x470); //gear packet with the ecu's calculated gear

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
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); //task runs every 50ms
    }
}
