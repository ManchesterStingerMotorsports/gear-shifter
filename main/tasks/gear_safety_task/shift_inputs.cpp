#include "shift_inputs.hpp"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG_SHIFT_INPUTS = "SHIFT_INPUTS";

static const gpio_num_t SHIFT_UP_GPIO = GPIO_NUM_45;
static const gpio_num_t SHIFT_DOWN_GPIO = GPIO_NUM_47;
static const gpio_num_t SHIFT_NEUTRAL_GPIO = GPIO_NUM_48;

static const uint32_t SHIFT_ISR_BLOCK_TIME_US = 150000;

static portMUX_TYPE shift_isr_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR block_for_shift_duration()
{
    portENTER_CRITICAL_ISR(&shift_isr_mux);
    esp_rom_delay_us(SHIFT_ISR_BLOCK_TIME_US);
    portEXIT_CRITICAL_ISR(&shift_isr_mux);
}

void IRAM_ATTR shift_up_isr(void *arg)
{
    (void)arg;
    block_for_shift_duration();
}

void IRAM_ATTR shift_down_isr(void *arg)
{
    (void)arg;
    block_for_shift_duration();
}

void IRAM_ATTR shift_neutral_isr(void *arg)
{
    (void)arg;
    block_for_shift_duration();
}

void setup_shift_inputs()
{
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

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
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
