#include "ESC.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUTTON_TEST";

static const gpio_num_t SHIFT_UP_GPIO = GPIO_NUM_48;
static const gpio_num_t SHIFT_DOWN_GPIO = GPIO_NUM_45;
static const gpio_num_t SHIFT_NEUTRAL_GPIO = GPIO_NUM_47;

static const gpio_num_t UP_LED_GPIO = GPIO_NUM_7;
static const gpio_num_t DOWN_LED_GPIO = GPIO_NUM_6;
static const gpio_num_t NEUTRAL_LED_GPIO = GPIO_NUM_5;

static const gpio_num_t ESC_PWM_GPIO = GPIO_NUM_9;

static constexpr uint32_t POLL_INTERVAL_MS = 20;
static constexpr uint32_t DEBOUNCE_MS = 60;
static constexpr float DOWN_TEST_TORQUE = -0.15f;
static constexpr bool LED_ACTIVE_HIGH = true;

static constexpr uint64_t gpio_select(gpio_num_t gpio)
{
    return 1ULL << static_cast<uint64_t>(gpio);
}

struct ButtonState
{
    bool up;
    bool down;
    bool neutral;
};

static void setup_inputs()
{
    gpio_config_t input_config = {};
    input_config.pin_bit_mask =
        gpio_select(SHIFT_DOWN_GPIO) |
        gpio_select(SHIFT_NEUTRAL_GPIO);
    input_config.mode = GPIO_MODE_INPUT;
    input_config.pull_up_en = GPIO_PULLUP_ENABLE;
    input_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&input_config));
}

static void setup_outputs()
{
    gpio_config_t output_config = {};
    output_config.pin_bit_mask =
        gpio_select(UP_LED_GPIO) |
        gpio_select(DOWN_LED_GPIO) |
        gpio_select(NEUTRAL_LED_GPIO);
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_DISABLE;
    output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&output_config));

    const int off_level = LED_ACTIVE_HIGH ? 0 : 1;
    ESP_ERROR_CHECK(gpio_set_level(UP_LED_GPIO, off_level));
    ESP_ERROR_CHECK(gpio_set_level(DOWN_LED_GPIO, off_level));
    ESP_ERROR_CHECK(gpio_set_level(NEUTRAL_LED_GPIO, off_level));
}

static bool button_pressed(gpio_num_t gpio)
{
    return gpio_get_level(gpio) == 0;
}

static void set_led(gpio_num_t gpio, bool on)
{
    const bool level = LED_ACTIVE_HIGH ? on : !on;
    ESP_ERROR_CHECK(gpio_set_level(gpio, level ? 1 : 0));
}

static void set_leds(const ButtonState &state)
{
    set_led(UP_LED_GPIO, false);
    set_led(DOWN_LED_GPIO, state.down);
    set_led(NEUTRAL_LED_GPIO, state.neutral);
}

static float torque_for_buttons(const ButtonState &state)
{
    if (state.down)
    {
        return DOWN_TEST_TORQUE;
    }

    return 0.0f;
}

static void set_esc_from_buttons(ESC &esc, const ButtonState &state)
{
    esc.set_torque(torque_for_buttons(state));
}

static ButtonState read_buttons()
{
    return {
        false,
        button_pressed(SHIFT_DOWN_GPIO),
        button_pressed(SHIFT_NEUTRAL_GPIO),
    };
}

static bool states_differ(const ButtonState &left, const ButtonState &right)
{
    return left.up != right.up ||
           left.down != right.down ||
           left.neutral != right.neutral;
}

static void log_buttons(const ButtonState &state)
{
    ESP_LOGI(TAG,
             "UP=%s DOWN=%s NEUTRAL=%s",
             state.up ? "PRESSED" : "released",
             state.down ? "PRESSED" : "released",
             state.neutral ? "PRESSED" : "released");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Button and ESC test started");
    ESP_LOGI(TAG,
             "Active-low inputs: DOWN GPIO=%d, NEUTRAL GPIO=%d; UP disabled",
             static_cast<int>(SHIFT_DOWN_GPIO),
             static_cast<int>(SHIFT_NEUTRAL_GPIO));
    ESP_LOGI(TAG,
             "LED outputs: UP GPIO=%d, DOWN GPIO=%d, NEUTRAL GPIO=%d",
             static_cast<int>(UP_LED_GPIO),
             static_cast<int>(DOWN_LED_GPIO),
             static_cast<int>(NEUTRAL_LED_GPIO));
    ESP_LOGI(TAG,
             "ESC PWM GPIO=%d, DOWN torque=%d/1000; UP torque disabled",
             static_cast<int>(ESC_PWM_GPIO),
             static_cast<int>(DOWN_TEST_TORQUE * 1000.0f));

    setup_outputs();
    setup_inputs();

    ESC esc(ESC_PWM_GPIO);

    ButtonState debounced_state = read_buttons();
    ButtonState candidate_state = debounced_state;
    uint32_t candidate_ticks = 0;

    set_leds(debounced_state);
    set_esc_from_buttons(esc, debounced_state);
    log_buttons(debounced_state);

    for (;;)
    {
        const ButtonState state = read_buttons();
        if (states_differ(state, candidate_state))
        {
            candidate_state = state;
            candidate_ticks = 0;
        }
        else if (states_differ(candidate_state, debounced_state))
        {
            candidate_ticks += POLL_INTERVAL_MS;
            if (candidate_ticks >= DEBOUNCE_MS)
            {
                debounced_state = candidate_state;
                candidate_ticks = 0;
                set_leds(debounced_state);
                set_esc_from_buttons(esc, debounced_state);
                log_buttons(debounced_state);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
