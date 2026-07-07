#include "shifter_task.hpp"

#include "ESC.hpp"
#include "config.hpp"
#include "shift_inputs.hpp"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_SHIFTER = "SHIFTER_TASK";

static constexpr uint64_t gpio_select(gpio_num_t gpio)
{
    return 1ULL << static_cast<uint64_t>(gpio);
}

static const char *shift_request_to_string(ShiftRequest request)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return "UP";
    case SHIFT_REQUEST_DOWN:
        return "DOWN";
    case SHIFT_REQUEST_NONE:
    default:
        return "NONE";
    }
}

static gpio_num_t led_for_shift_request(ShiftRequest request)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return config::UP_LED_GPIO;
    case SHIFT_REQUEST_DOWN:
        return config::DOWN_LED_GPIO;
    case SHIFT_REQUEST_NONE:
    default:
        return GPIO_NUM_NC;
    }
}

static float torque_for_shift_request(ShiftRequest request)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return config::SHIFT_UP_TORQUE;
    case SHIFT_REQUEST_DOWN:
        return config::SHIFT_DOWN_TORQUE;
    case SHIFT_REQUEST_NONE:
    default:
        return 0.0f;
    }
}

static void set_led(gpio_num_t gpio, bool on)
{
    if (gpio == GPIO_NUM_NC)
    {
        return;
    }

    const bool level = config::LED_ACTIVE_HIGH ? on : !on;
    ESP_ERROR_CHECK(gpio_set_level(gpio, level ? 1 : 0));
}

static void set_all_leds(bool on)
{
    set_led(config::UP_LED_GPIO, on);
    set_led(config::DOWN_LED_GPIO, on);
    set_led(config::NEUTRAL_LED_GPIO, on);
}

static void setup_shift_leds()
{
    gpio_config_t output_config = {};
    output_config.pin_bit_mask =
        gpio_select(config::UP_LED_GPIO) |
        gpio_select(config::DOWN_LED_GPIO) |
        gpio_select(config::NEUTRAL_LED_GPIO);
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_DISABLE;
    output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&output_config));
    set_all_leds(false);
}

static void process_shift_request(ShiftRequest request, ESC &esc)
{
    if (request != SHIFT_REQUEST_UP && request != SHIFT_REQUEST_DOWN)
    {
        ESP_LOGI(TAG_SHIFTER,
                 "Ignoring shift request: request=%s",
                 shift_request_to_string(request));
        esc.set_torque(0.0f);
        return;
    }

    const float shift_torque = torque_for_shift_request(request);
    const int shift_torque_milli = static_cast<int>(shift_torque * 1000.0f);
    const gpio_num_t led_gpio = led_for_shift_request(request);

    ESP_LOGI(TAG_SHIFTER,
             "Shift pulse: request=%s torque_milli=%d duration_ms=%u",
             shift_request_to_string(request),
             shift_torque_milli,
             static_cast<unsigned>(config::SHIFT_ACTUATION_MS));

    set_led(led_gpio, true);
    esc.set_torque(shift_torque);
    vTaskDelay(pdMS_TO_TICKS(config::SHIFT_ACTUATION_MS));
    esc.set_torque(0.0f);
    set_led(led_gpio, false);

    ESP_LOGI(TAG_SHIFTER, "Shift pulse complete: ESC neutral");
}

void shifter_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG_SHIFTER, "Automatic alternating shifter test started");

    setup_shift_leds();
    ESC esc(config::ESC_PWM_GPIO);

    esc.set_torque(0.0f);
    ESP_LOGI(TAG_SHIFTER, "ESC neutral command sent: waiting 2 seconds to arm");
    vTaskDelay(pdMS_TO_TICKS(config::AUTO_SHIFT_PERIOD_MS));

    ShiftRequest request = SHIFT_REQUEST_UP;
    for (;;)
    {
        process_shift_request(request, esc);
        request = request == SHIFT_REQUEST_UP
                      ? SHIFT_REQUEST_DOWN
                      : SHIFT_REQUEST_UP;

        vTaskDelay(pdMS_TO_TICKS(
            config::AUTO_SHIFT_PERIOD_MS - config::SHIFT_ACTUATION_MS));
    }
}
