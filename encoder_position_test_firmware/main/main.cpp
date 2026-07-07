#include "AMT20.hpp"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ENCODER_TEST";

static const gpio_num_t ENCODER_CS_GPIO = GPIO_NUM_10;
static const gpio_num_t ENCODER_MOSI_GPIO = GPIO_NUM_11;
static const gpio_num_t ENCODER_SCLK_GPIO = GPIO_NUM_12;
static const gpio_num_t ENCODER_MISO_GPIO = GPIO_NUM_13;

static const uint32_t POLL_INTERVAL_MS = 10;
static const uint32_t PRINT_INTERVAL_MS = 500;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Encoder position test started");

    AMT20 encoder(SPI2_HOST,
                  ENCODER_CS_GPIO,
                  ENCODER_MOSI_GPIO,
                  ENCODER_MISO_GPIO,
                  ENCODER_SCLK_GPIO);

    uint16_t last_position = 0;
    bool has_last_position = false;
    uint32_t read_ok_count = 0;
    uint32_t read_fail_count = 0;
    uint32_t print_ticks = PRINT_INTERVAL_MS;

    for (;;)
    {
        uint16_t position = 0;
        if (encoder.read_position(position))
        {
            last_position = position;
            has_last_position = true;
            ++read_ok_count;
        }
        else
        {
            ++read_fail_count;
        }

        if (print_ticks >= PRINT_INTERVAL_MS)
        {
            print_ticks = 0;
            if (has_last_position)
            {
                ESP_LOGI(TAG,
                         "position=%u reads_ok=%lu reads_failed=%lu",
                         static_cast<unsigned>(last_position),
                         static_cast<unsigned long>(read_ok_count),
                         static_cast<unsigned long>(read_fail_count));
            }
            else
            {
                ESP_LOGI(TAG,
                         "position=none reads_ok=%lu reads_failed=%lu",
                         static_cast<unsigned long>(read_ok_count),
                         static_cast<unsigned long>(read_fail_count));
            }

            read_ok_count = 0;
            read_fail_count = 0;
        }
        else
        {
            print_ticks += POLL_INTERVAL_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
