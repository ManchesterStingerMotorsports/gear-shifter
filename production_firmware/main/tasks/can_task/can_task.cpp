#include "can_task.hpp"

#include "MSM_CAN.hpp"
#include "config.hpp"

#include <cstdint>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

static const char *TAG_CAN_TASK = "CAN_TASK";

static constexpr uint16_t HALTECH_ID_SPEED_GEAR = 0x370;
static constexpr uint16_t HALTECH_ID_SWITCHES = 0x3E4;
static constexpr uint16_t SHIFTER_STATUS_TX_ID = 0x100;

static constexpr uint32_t CAN_POLL_INTERVAL_MS = 20;
static constexpr uint32_t SHIFTER_STATUS_TX_PERIOD_MS = 50;
static constexpr uint32_t SPEED_GEAR_TIMEOUT_MS = 250;
static constexpr uint32_t SWITCHES_TIMEOUT_MS = 1000;

static constexpr uint16_t MOVING_SPEED_RAW_THRESHOLD = 30;

static portMUX_TYPE can_task_output_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE shifter_task_status_mux = portMUX_INITIALIZER_UNLOCKED;

static CanTaskOutput can_task_output = {false, 0, false, false, false};
static ShifterTaskCanStatus shifter_task_status = {0, SHIFTER_CAN_STATUS_STARTUP, 0};
static TaskHandle_t can_task_handle = nullptr;

CanTaskOutput get_can_task_output()
{
    CanTaskOutput output;

    portENTER_CRITICAL(&can_task_output_mux);
    output = can_task_output;
    portEXIT_CRITICAL(&can_task_output_mux);

    return output;
}

void set_can_task_output(const CanTaskOutput &output)
{
    portENTER_CRITICAL(&can_task_output_mux);
    can_task_output = output;
    portEXIT_CRITICAL(&can_task_output_mux);
}

ShifterTaskCanStatus get_shifter_task_can_status()
{
    ShifterTaskCanStatus status;

    portENTER_CRITICAL(&shifter_task_status_mux);
    status = shifter_task_status;
    portEXIT_CRITICAL(&shifter_task_status_mux);

    return status;
}

void set_shifter_task_can_status(const ShifterTaskCanStatus &status)
{
    portENTER_CRITICAL(&shifter_task_status_mux);
    shifter_task_status = status;
    portEXIT_CRITICAL(&shifter_task_status_mux);
}

void suspend_can_tasks_for_shift()
{
    if (can_task_handle != nullptr)
    {
        vTaskSuspend(can_task_handle);
    }

    MSM_CAN::suspend_background_tasks();
}

void resume_can_tasks_after_shift()
{
    MSM_CAN::resume_background_tasks();

    if (can_task_handle != nullptr)
    {
        vTaskResume(can_task_handle);
    }
}

static uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

static bool frame_is_fresh(const MSM_CAN::RxFrame &frame, uint32_t max_age_ms)
{
    return static_cast<int32_t>(now_ms() - frame.timestamp_ms) <= static_cast<int32_t>(max_age_ms);
}

static CanTaskOutput read_haltech_output()
{
    CanTaskOutput output = {false, 0, false, false, false};

    MSM_CAN::RxFrame speed_gear_frame{};
    if (MSM_CAN::get(HALTECH_ID_SPEED_GEAR, speed_gear_frame) == ESP_OK &&
        frame_is_fresh(speed_gear_frame, SPEED_GEAR_TIMEOUT_MS))
    {
        output.speed_gear_valid = true;

        const uint16_t vehicle_speed_raw = MSM_CAN::unpack_u16(speed_gear_frame.data, 0);
        output.is_moving = vehicle_speed_raw > MOVING_SPEED_RAW_THRESHOLD;

        const uint16_t combined_gear = MSM_CAN::unpack_u16(speed_gear_frame.data, 2);
        output.ecu_gear = static_cast<int8_t>(combined_gear & 0xFF);
    }

    MSM_CAN::RxFrame switches_frame{};
    if (MSM_CAN::get(HALTECH_ID_SWITCHES, switches_frame) == ESP_OK &&
        frame_is_fresh(switches_frame, SWITCHES_TIMEOUT_MS))
    {
        output.switches_valid = true;
        output.neutral_pos = MSM_CAN::check_flag(switches_frame.data, 1, 7);
    }

    return output;
}

static bool initialise_haltech_can()
{
    MSM_CAN::set_hardware_filters(0x300, 0x3FF);

    esp_err_t err = MSM_CAN::init(config::CAN_RX_GPIO, config::CAN_TX_GPIO);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TASK, "MSM_CAN init failed: %s", esp_err_to_name(err));
        return false;
    }

    err = MSM_CAN::subscribe(HALTECH_ID_SPEED_GEAR);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TASK, "Failed to subscribe to Haltech speed/gear frame 0x%03X: %s",
                 HALTECH_ID_SPEED_GEAR,
                 esp_err_to_name(err));
        return false;
    }

    err = MSM_CAN::subscribe(HALTECH_ID_SWITCHES);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TASK, "Failed to subscribe to Haltech switches frame 0x%03X: %s",
                 HALTECH_ID_SWITCHES,
                 esp_err_to_name(err));
        return false;
    }

    MSM_CAN::TxFrame status_frame{};
    status_frame.id = SHIFTER_STATUS_TX_ID;
    MSM_CAN::clear_payload(status_frame.data);
    err = MSM_CAN::schedule(status_frame, SHIFTER_STATUS_TX_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CAN_TASK, "Failed to schedule shifter status frame 0x%03X: %s",
                 SHIFTER_STATUS_TX_ID,
                 esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG_CAN_TASK,
             "Haltech CAN subscribed: speed/gear=0x%03X switches=0x%03X status_tx=0x%03X rx=IO%d tx=IO%d",
             HALTECH_ID_SPEED_GEAR,
             HALTECH_ID_SWITCHES,
             SHIFTER_STATUS_TX_ID,
             static_cast<int>(config::CAN_RX_GPIO),
             static_cast<int>(config::CAN_TX_GPIO));
    return true;
}

static MSM_CAN::TxFrame build_shifter_status_frame()
{
    const ShifterTaskCanStatus shifter_status = get_shifter_task_can_status();
    const CanTaskOutput can_output = get_can_task_output();

    MSM_CAN::TxFrame frame{};
    frame.id = SHIFTER_STATUS_TX_ID;
    MSM_CAN::clear_payload(frame.data);

    MSM_CAN::pack_i8(frame.data, 0, shifter_status.internal_gear);
    MSM_CAN::pack_u8(frame.data, 1, shifter_status.shifter_status);
    MSM_CAN::pack_u8(frame.data, 2, shifter_status.last_request);

    uint8_t flags = 0;
    MSM_CAN::set_bit(flags, 0, can_output.neutral_pos);
    MSM_CAN::set_bit(flags, 1, can_output.is_moving);
    MSM_CAN::set_bit(flags, 2, can_output.speed_gear_valid && can_output.ecu_gear >= 1 && can_output.ecu_gear <= 5);
    MSM_CAN::set_bit(flags, 3, config::STANDALONE_TESTING);
    MSM_CAN::set_bit(flags, 4, can_output.speed_gear_valid);
    MSM_CAN::set_bit(flags, 5, can_output.switches_valid);
    MSM_CAN::pack_u8(frame.data, 3, flags);

    MSM_CAN::pack_i8(frame.data, 4, can_output.ecu_gear);
    return frame;
}

void can_task(void *arg)
{
    (void)arg;
    can_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG_CAN_TASK, "CAN task started");
    set_can_task_output({false, 0, false, false, false});

    if (!initialise_haltech_can())
    {
        for (;;)
        {
            set_can_task_output({false, 0, false, false, false});
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    for (;;)
    {
        set_can_task_output(read_haltech_output());
        (void)MSM_CAN::update_scheduled_payload(build_shifter_status_frame());
        vTaskDelay(pdMS_TO_TICKS(CAN_POLL_INTERVAL_MS));
    }
}
