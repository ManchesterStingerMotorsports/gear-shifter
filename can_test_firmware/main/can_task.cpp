#include "can_task.hpp"

#include "MSM_CAN.hpp"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_CAN_TEST = "CAN_TEST";

static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_2;
static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_1;

static constexpr uint16_t HALTECH_ID_SPEED_GEAR = 0x370;
static constexpr uint16_t HALTECH_ID_SWITCHES = 0x3E4;

static constexpr uint32_t CAN_POLL_INTERVAL_MS = 20;

void can_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_CAN_TEST, "CAN task started");

    MSM_CAN::set_hardware_filters(0x300, 0x3FF);

    esp_err_t err = MSM_CAN::init(CAN_RX_GPIO, CAN_TX_GPIO);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TEST, "MSM_CAN init failed: %s", esp_err_to_name(err));
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = MSM_CAN::subscribe(HALTECH_ID_SPEED_GEAR);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TEST,
                 "Failed to subscribe to speed/gear frame 0x%03X: %s",
                 static_cast<unsigned int>(HALTECH_ID_SPEED_GEAR),
                 esp_err_to_name(err));
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = MSM_CAN::subscribe(HALTECH_ID_SWITCHES);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TEST,
                 "Failed to subscribe to switches frame 0x%03X: %s",
                 static_cast<unsigned int>(HALTECH_ID_SWITCHES),
                 esp_err_to_name(err));
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG_CAN_TEST,
             "Listening for Haltech frames: speed/gear=0x%03X switches=0x%03X rx=IO%d tx=IO%d",
             static_cast<unsigned int>(HALTECH_ID_SPEED_GEAR),
             static_cast<unsigned int>(HALTECH_ID_SWITCHES),
             static_cast<int>(CAN_RX_GPIO),
             static_cast<int>(CAN_TX_GPIO));

    uint32_t last_speed_gear_timestamp_ms = 0;
    uint32_t last_switches_timestamp_ms = 0;

    for (;;)
    {
        MSM_CAN::RxFrame speed_gear_frame{};
        if (MSM_CAN::get(HALTECH_ID_SPEED_GEAR, speed_gear_frame) == ESP_OK &&
            speed_gear_frame.timestamp_ms != last_speed_gear_timestamp_ms)
        {
            last_speed_gear_timestamp_ms = speed_gear_frame.timestamp_ms;

            const uint16_t vehicle_speed_raw = MSM_CAN::unpack_u16(speed_gear_frame.data, 0);
            const uint16_t combined_gear = MSM_CAN::unpack_u16(speed_gear_frame.data, 2);
            const int8_t ecu_gear = static_cast<int8_t>(combined_gear & 0xFF);

            ESP_LOGI(TAG_CAN_TEST,
                     "0x%03X speed_raw=%u ecu_gear=%d data=%02X %02X %02X %02X %02X %02X %02X %02X ts=%lu",
                     static_cast<unsigned int>(speed_gear_frame.id),
                     static_cast<unsigned int>(vehicle_speed_raw),
                     static_cast<int>(ecu_gear),
                     static_cast<unsigned int>(speed_gear_frame.data[0]),
                     static_cast<unsigned int>(speed_gear_frame.data[1]),
                     static_cast<unsigned int>(speed_gear_frame.data[2]),
                     static_cast<unsigned int>(speed_gear_frame.data[3]),
                     static_cast<unsigned int>(speed_gear_frame.data[4]),
                     static_cast<unsigned int>(speed_gear_frame.data[5]),
                     static_cast<unsigned int>(speed_gear_frame.data[6]),
                     static_cast<unsigned int>(speed_gear_frame.data[7]),
                     static_cast<unsigned long>(speed_gear_frame.timestamp_ms));
        }

        MSM_CAN::RxFrame switches_frame{};
        if (MSM_CAN::get(HALTECH_ID_SWITCHES, switches_frame) == ESP_OK &&
            switches_frame.timestamp_ms != last_switches_timestamp_ms)
        {
            last_switches_timestamp_ms = switches_frame.timestamp_ms;

            const bool neutral_pos = MSM_CAN::check_flag(switches_frame.data, 1, 7);

            ESP_LOGI(TAG_CAN_TEST,
                     "0x%03X neutral_pos=%d data=%02X %02X %02X %02X %02X %02X %02X %02X ts=%lu",
                     static_cast<unsigned int>(switches_frame.id),
                     neutral_pos ? 1 : 0,
                     static_cast<unsigned int>(switches_frame.data[0]),
                     static_cast<unsigned int>(switches_frame.data[1]),
                     static_cast<unsigned int>(switches_frame.data[2]),
                     static_cast<unsigned int>(switches_frame.data[3]),
                     static_cast<unsigned int>(switches_frame.data[4]),
                     static_cast<unsigned int>(switches_frame.data[5]),
                     static_cast<unsigned int>(switches_frame.data[6]),
                     static_cast<unsigned int>(switches_frame.data[7]),
                     static_cast<unsigned long>(switches_frame.timestamp_ms));
        }

        vTaskDelay(pdMS_TO_TICKS(CAN_POLL_INTERVAL_MS));
    }
}
