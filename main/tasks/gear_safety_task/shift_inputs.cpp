#include "shift_inputs.hpp"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

static const char *TAG_SHIFT_INPUTS = "SHIFT_INPUTS";

static const gpio_num_t SHIFT_UP_GPIO = GPIO_NUM_45;
static const gpio_num_t SHIFT_DOWN_GPIO = GPIO_NUM_47;
static const gpio_num_t SHIFT_NEUTRAL_GPIO = GPIO_NUM_48;

// Protects the single pending request shared between the ISR and task.
static portMUX_TYPE shift_input_mux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t shift_task_handle = nullptr;

// First request wins. Further inputs are ignored until the task re-enables GPIO interrupts.
static volatile ShiftRequest pending_shift_request = SHIFT_REQUEST_NONE;

static void disable_shift_input_interrupts()
{
    gpio_intr_disable(SHIFT_UP_GPIO);
    gpio_intr_disable(SHIFT_DOWN_GPIO);
    gpio_intr_disable(SHIFT_NEUTRAL_GPIO);
}

static void enable_shift_input_interrupts()
{
    gpio_intr_enable(SHIFT_UP_GPIO);
    gpio_intr_enable(SHIFT_DOWN_GPIO);
    gpio_intr_enable(SHIFT_NEUTRAL_GPIO);
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
    // Clear stale/bounced requests before accepting another shift input.
    portENTER_CRITICAL(&shift_input_mux);
    pending_shift_request = SHIFT_REQUEST_NONE;
    portEXIT_CRITICAL(&shift_input_mux);

    enable_shift_input_interrupts();
}

void setup_shift_inputs(TaskHandle_t task_handle)
{
    shift_task_handle = task_handle;

    gpio_config_t shift_input_config = {};
    shift_input_config.pin_bit_mask =
        GPIO_SEL_45 |
        GPIO_SEL_47 |
        GPIO_SEL_48;
    shift_input_config.mode = GPIO_MODE_INPUT;
    shift_input_config.pull_up_en = GPIO_PULLUP_DISABLE;
    shift_input_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    shift_input_config.intr_type = GPIO_INTR_POSEDGE;

    esp_err_t err = gpio_config(&shift_input_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "GPIO config failed: %s", esp_err_to_name(err));
        return;
    }

    // ESP_ERR_INVALID_STATE means another module already installed the ISR service.
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "ISR service install failed: %s", esp_err_to_name(err));
        return;
    }

    err = gpio_isr_handler_add(SHIFT_UP_GPIO, shift_up_isr, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "UP ISR add failed: %s", esp_err_to_name(err));
        return;
    }

    err = gpio_isr_handler_add(SHIFT_DOWN_GPIO, shift_down_isr, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "DOWN ISR add failed: %s", esp_err_to_name(err));
        return;
    }

    err = gpio_isr_handler_add(SHIFT_NEUTRAL_GPIO, shift_neutral_isr, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFT_INPUTS, "NEUTRAL ISR add failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG_SHIFT_INPUTS, "Shift inputs configured on IO45, IO47, IO48");
}
