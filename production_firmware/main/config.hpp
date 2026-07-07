#pragma once

#include <cstdint>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace config
{
static constexpr bool STANDALONE_TESTING = true;
static constexpr int8_t STANDALONE_INITIAL_GEAR = 2;

// Keep the shifter actuation path on one core so priority, not cross-core
// scheduling, decides what can run during the shift actuation window.
static const BaseType_t CONTROL_CORE = 1;

static const UBaseType_t SHIFTER_TASK_PRIORITY = configMAX_PRIORITIES - 1;

static constexpr uint32_t SHIFTER_TASK_STACK_WORDS = 4096;

static constexpr gpio_num_t ESC_PWM_GPIO = GPIO_NUM_9;

static constexpr gpio_num_t SHIFT_UP_GPIO = GPIO_NUM_47;
static constexpr gpio_num_t SHIFT_DOWN_GPIO = GPIO_NUM_48;

static constexpr gpio_num_t UP_LED_GPIO = GPIO_NUM_7;
static constexpr gpio_num_t DOWN_LED_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t NEUTRAL_LED_GPIO = GPIO_NUM_5;
static constexpr bool LED_ACTIVE_HIGH = true;

static constexpr uint32_t SHIFT_INPUT_RELEASE_DEBOUNCE_MS = 100;
static constexpr uint32_t SHIFT_INPUT_RELEASE_POLL_MS = 1;

static constexpr uint32_t SHIFT_ACTUATION_MS = 200;
static constexpr uint32_t AUTO_SHIFT_PERIOD_MS = 2000;

static constexpr float SHIFT_UP_TORQUE = 0.8f;
static constexpr float SHIFT_DOWN_TORQUE = -0.8f;
}
