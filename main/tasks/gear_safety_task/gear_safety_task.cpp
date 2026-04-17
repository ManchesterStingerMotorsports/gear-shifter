#include "gear_safety_task.hpp"
#include "shift_inputs.hpp"

#include "copperhead10_esc.hpp"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG_GEAR_SAFETY = "GEAR_SAFETY_TASK";

enum gear_t {GEAR_N, GEAR_1, GEAR_2, GEAR_3, GEAR_4, GEAR_5};

static const gpio_num_t ESC_PWM_GPIO = GPIO_NUM_9;

void gear_safety_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG_GEAR_SAFETY, "Gear safety task started");

    Copperhead10Esc esc(ESC_PWM_GPIO);
    esc.set_torque(0.0f);

    setup_shift_inputs();

    gear_t gear_count = GEAR_N;
    

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
