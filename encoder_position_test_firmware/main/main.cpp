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

static const uint32_t PRINT_INTERVAL_MS = 100;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Encoder position test started");

    AMT20 encoder(SPI2_HOST,
                  ENCODER_CS_GPIO,
                  ENCODER_MOSI_GPIO,
                  ENCODER_MISO_GPIO,
                  ENCODER_SCLK_GPIO);

    for (;;)
    {
        uint16_t position = 0;
        if (encoder.read_position(position))
        {
            ESP_LOGI(TAG, "position=%u", static_cast<unsigned>(position));
        }
        else
        {
            ESP_LOGW(TAG, "position read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));
    }
}
