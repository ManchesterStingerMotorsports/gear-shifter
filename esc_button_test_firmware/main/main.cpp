#include "ESC.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ESC_TEST";

static const gpio_num_t ESC_PWM_GPIO = GPIO_NUM_9;
static const gpio_num_t SHIFT_UP_GPIO = GPIO_NUM_47;
static const gpio_num_t SHIFT_DOWN_GPIO = GPIO_NUM_45;

// Change these for bench testing. Range is -1.0f to 1.0f.
static const float UP_TEST_TORQUE = 0.4f;
static const float DOWN_TEST_TORQUE = -0.4f;

static const uint32_t POLL_INTERVAL_MS = 10;

static constexpr uint64_t gpio_select(gpio_num_t gpio)
{
    return 1ULL << static_cast<uint64_t>(gpio);
}

static void setup_inputs()
{
    gpio_config_t input_config = {};
    input_config.pin_bit_mask = gpio_select(SHIFT_UP_GPIO) | gpio_select(SHIFT_DOWN_GPIO);
    input_config.mode = GPIO_MODE_INPUT;
    input_config.pull_up_en = GPIO_PULLUP_DISABLE;
    input_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    input_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&input_config));
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "ESC button test started");
    ESP_LOGI(TAG,
             "UP GPIO=%d torque=%d/1000, DOWN GPIO=%d torque=%d/1000",
             static_cast<int>(SHIFT_UP_GPIO),
             static_cast<int>(UP_TEST_TORQUE * 1000.0f),
             static_cast<int>(SHIFT_DOWN_GPIO),
             static_cast<int>(DOWN_TEST_TORQUE * 1000.0f));

    setup_inputs();

    ESC esc(ESC_PWM_GPIO);
    esc.set_torque(0.0f);

    float last_torque = 99.0f;

    for (;;)
    {
        const bool up_pressed = gpio_get_level(SHIFT_UP_GPIO) != 0;
        const bool down_pressed = gpio_get_level(SHIFT_DOWN_GPIO) != 0;

        float torque = 0.0f;
        if (up_pressed && !down_pressed)
        {
            torque = UP_TEST_TORQUE;
        }
        else if (down_pressed && !up_pressed)
        {
            torque = DOWN_TEST_TORQUE;
        }

        if (torque != last_torque)
        {
            esc.set_torque(torque);
            last_torque = torque;
            ESP_LOGI(TAG, "Torque command=%d/1000", static_cast<int>(torque * 1000.0f));
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
