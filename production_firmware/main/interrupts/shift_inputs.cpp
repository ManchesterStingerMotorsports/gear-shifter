#include "shift_inputs.hpp"

#include "config.hpp"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

static const char *TAG_SHIFT_INPUTS = "SHIFT_INPUTS";

static constexpr uint64_t gpio_select(gpio_num_t gpio)
{
    return 1ULL << static_cast<uint64_t>(gpio);
}

static TickType_t ticks_at_least_one(uint32_t milliseconds)
{
    const TickType_t ticks = pdMS_TO_TICKS(milliseconds);
    return ticks > 0 ? ticks : 1;
}

// Protects the single pending request shared between the ISR and task.
static portMUX_TYPE shift_input_mux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t shift_task_handle = nullptr;

// First request wins. Further inputs are ignored until the task re-enables GPIO interrupts.
static volatile ShiftRequest pending_shift_request = SHIFT_REQUEST_NONE;

static void disable_shift_input_interrupts()
{
    gpio_intr_disable(config::SHIFT_UP_GPIO);
    gpio_intr_disable(config::SHIFT_DOWN_GPIO);
    gpio_intr_disable(config::SHIFT_NEUTRAL_GPIO);
}

static void enable_shift_input_interrupts()
{
    gpio_intr_enable(config::SHIFT_UP_GPIO);
    gpio_intr_enable(config::SHIFT_DOWN_GPIO);
    gpio_intr_enable(config::SHIFT_NEUTRAL_GPIO);
}

static bool shift_inputs_released()
{
    return gpio_get_level(config::SHIFT_UP_GPIO) == 0 &&
           gpio_get_level(config::SHIFT_DOWN_GPIO) == 0 &&
           gpio_get_level(config::SHIFT_NEUTRAL_GPIO) == 0;
}

static void wait_for_shift_inputs_released()
{
    const TickType_t debounce_ticks = ticks_at_least_one(config::SHIFT_INPUT_RELEASE_DEBOUNCE_MS);
    const TickType_t poll_ticks = ticks_at_least_one(config::SHIFT_INPUT_RELEASE_POLL_MS);
    TickType_t released_since = 0;
    bool released = false;

    while (true)
    {
        const TickType_t now = xTaskGetTickCount();

        if (shift_inputs_released())
        {
            if (!released)
            {
                released_since = now;
                released = true;
            }

            if ((now - released_since) >= debounce_ticks)
            {
                return;
            }
        }
        else
        {
            released = false;
            released_since = 0;
        }

        vTaskDelay(poll_ticks);
    }
}

// Latch a request and wake the shift task. The shift itself is not done here.
static void IRAM_ATTR shift_input_isr(ShiftRequest request)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    bool accepted = false;

    portENTER_CRITICAL_ISR(&shift_input_mux);

    if (pending_shift_request == SHIFT_REQUEST_NONE && shift_task_handle != nullptr)
    {
        pending_shift_request = request;
        accepted = true;
    }
    portEXIT_CRITICAL_ISR(&shift_input_mux);

    if (!accepted)
    {
        return;
    }

    disable_shift_input_interrupts();
    vTaskNotifyGiveFromISR(shift_task_handle, &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR shift_up_isr(void *arg)
{
    (void)arg;
    shift_input_isr(SHIFT_REQUEST_UP);
}

void IRAM_ATTR shift_down_isr(void *arg)
{
    (void)arg;
    shift_input_isr(SHIFT_REQUEST_DOWN);
}

void IRAM_ATTR shift_neutral_isr(void *arg)
{
    (void)arg;
    shift_input_isr(SHIFT_REQUEST_NEUTRAL);
}

ShiftRequest consume_pending_shift_request()
{
    ShiftRequest request = SHIFT_REQUEST_NONE;

    portENTER_CRITICAL(&shift_input_mux);
    request = pending_shift_request;
    pending_shift_request = SHIFT_REQUEST_NONE;
    portEXIT_CRITICAL(&shift_input_mux);

    return request;
}

void clear_and_enable_shift_inputs()
{
    // Clear stale requests before waiting for the physical switches to release.
    portENTER_CRITICAL(&shift_input_mux);
    pending_shift_request = SHIFT_REQUEST_NONE;
    portEXIT_CRITICAL(&shift_input_mux);

    wait_for_shift_inputs_released();

    // Discard anything observed while the release debounce was settling.
    portENTER_CRITICAL(&shift_input_mux);
    pending_shift_request = SHIFT_REQUEST_NONE;
    portEXIT_CRITICAL(&shift_input_mux);

    enable_shift_input_interrupts();
}

esp_err_t setup_shift_inputs(TaskHandle_t task_handle)
{
    shift_task_handle = task_handle;

    gpio_config_t shift_input_config = {};
    shift_input_config.pin_bit_mask =
        gpio_select(config::SHIFT_UP_GPIO) |
        gpio_select(config::SHIFT_DOWN_GPIO) |
        gpio_select(config::SHIFT_NEUTRAL_GPIO);
    shift_input_config.mode = GPIO_MODE_INPUT;
    shift_input_config.pull_up_en = GPIO_PULLUP_DISABLE;
    shift_input_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    shift_input_config.intr_type = GPIO_INTR_POSEDGE;

    esp_err_t err = gpio_config(&shift_input_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    // ESP_ERR_INVALID_STATE means another module already installed the ISR service.
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "ISR service install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(config::SHIFT_UP_GPIO, shift_up_isr, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "UP ISR add failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(config::SHIFT_DOWN_GPIO, shift_down_isr, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "DOWN ISR add failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(config::SHIFT_NEUTRAL_GPIO, shift_neutral_isr, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "NEUTRAL ISR add failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG_SHIFT_INPUTS, "Shift inputs configured on IO45, IO47, IO48");
    return ESP_OK;
}
