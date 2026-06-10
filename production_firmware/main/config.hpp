#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace config
{
static constexpr bool STANDALONE_TESTING = false;

// Keep control-path tasks on one core so priority, not cross-core scheduling,
// decides what can run during the shift actuation window.
static const BaseType_t CONTROL_CORE = 1;

static const UBaseType_t SHIFTER_TASK_PRIORITY = configMAX_PRIORITIES - 1;
static const UBaseType_t CAN_TASK_PRIORITY = configMAX_PRIORITIES - 2;

static constexpr uint32_t SHIFTER_TASK_STACK_WORDS = 4096;
static constexpr uint32_t CAN_TASK_STACK_WORDS = 3072;

static constexpr gpio_num_t ESC_PWM_GPIO = GPIO_NUM_9;

static constexpr spi_host_device_t ENCODER_SPI_HOST = SPI2_HOST;
static constexpr gpio_num_t ENCODER_CS_GPIO = GPIO_NUM_10;
static constexpr gpio_num_t ENCODER_MOSI_GPIO = GPIO_NUM_11;
static constexpr gpio_num_t ENCODER_SCLK_GPIO = GPIO_NUM_12;
static constexpr gpio_num_t ENCODER_MISO_GPIO = GPIO_NUM_13;

static constexpr gpio_num_t SHIFT_UP_GPIO = GPIO_NUM_47;
static constexpr gpio_num_t SHIFT_DOWN_GPIO = GPIO_NUM_45;
static constexpr gpio_num_t SHIFT_NEUTRAL_GPIO = GPIO_NUM_48;

static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_2;
static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_1;

static constexpr uint32_t SHIFT_INPUT_RELEASE_DEBOUNCE_MS = 30;
static constexpr uint32_t SHIFT_INPUT_RELEASE_POLL_MS = 1;

static constexpr int64_t SHIFT_TIMEOUT_US = 200000;

static constexpr float SHIFT_UP_TORQUE = 0.4f;
static constexpr float SHIFT_DOWN_TORQUE = -0.4f;
static constexpr float SHIFT_1_TO_NEUTRAL_TORQUE = SHIFT_UP_TORQUE;
static constexpr float SHIFT_NEUTRAL_TO_1_TORQUE = SHIFT_DOWN_TORQUE;

static constexpr uint16_t BASE_POSITION = 2502;
static constexpr uint16_t SHIFT_UP_STOP_POSITION = 2290;
static constexpr uint16_t SHIFT_DOWN_STOP_POSITION = 2670;
static constexpr uint16_t SHIFT_1_TO_NEUTRAL_STOP_POSITION = BASE_POSITION;
static constexpr uint16_t SHIFT_NEUTRAL_TO_1_STOP_POSITION = SHIFT_DOWN_STOP_POSITION;
static constexpr uint16_t SHIFT_POSITION_TOLERANCE = 20;

static const TickType_t ENCODER_IDLE_LOG_INTERVAL = pdMS_TO_TICKS(100);
}
