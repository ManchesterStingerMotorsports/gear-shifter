#include "shifter_task.hpp"

#include "AMT20.hpp"
#include "ESC.hpp"
#include "can_task.hpp"
#include "config.hpp"
#include "shift_inputs.hpp"

#include <cstdint>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_SHIFTER = "SHIFTER_TASK";

enum Gear
{
    GEAR_N = 0,
    GEAR_1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,
};

enum class ShiftSafetyStatus
{
    Allowed,
    InvalidRequest,
    OutOfRange,
    NeutralUnavailable,
    CanUnavailable,
    EncoderFault,
};

enum class ShiftPositionStatus
{
    NotReached,
    Reached,
    ReadFault,
};

static const char *shift_request_to_string(ShiftRequest request)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return "UP";
    case SHIFT_REQUEST_DOWN:
        return "DOWN";
    case SHIFT_REQUEST_NEUTRAL:
        return "NEUTRAL";
    case SHIFT_REQUEST_NONE:
    default:
        return "NONE";
    }
}

static const char *gear_to_string(Gear gear)
{
    switch (gear)
    {
    case GEAR_N:
        return "N";
    case GEAR_1:
        return "1";
    case GEAR_2:
        return "2";
    case GEAR_3:
        return "3";
    case GEAR_4:
        return "4";
    case GEAR_5:
        return "5";
    default:
        return "?";
    }
}

static bool ecu_gear_to_gear(int8_t ecu_gear, Gear &gear)
{
    switch (ecu_gear)
    {
    case 1:
        gear = GEAR_1;
        return true;
    case 2:
        gear = GEAR_2;
        return true;
    case 3:
        gear = GEAR_3;
        return true;
    case 4:
        gear = GEAR_4;
        return true;
    case 5:
        gear = GEAR_5;
        return true;
    default:
        return false;
    }
}

static const char *shift_safety_status_to_string(ShiftSafetyStatus status)
{
    switch (status)
    {
    case ShiftSafetyStatus::Allowed:
        return "allowed";
    case ShiftSafetyStatus::InvalidRequest:
        return "invalid request";
    case ShiftSafetyStatus::OutOfRange:
        return "shift would exceed gearbox range";
    case ShiftSafetyStatus::NeutralUnavailable:
        return "neutral is only available from 1st";
    case ShiftSafetyStatus::CanUnavailable:
        return "CAN safety data unavailable";
    case ShiftSafetyStatus::EncoderFault:
        return "encoder pre-check failed";
    default:
        return "unknown";
    }
}

static ShiftSafetyStatus shift_request_safety_status(ShiftRequest request, Gear gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return gear_count == GEAR_5 ? ShiftSafetyStatus::OutOfRange : ShiftSafetyStatus::Allowed;
    case SHIFT_REQUEST_DOWN:
        if (gear_count == GEAR_N || gear_count == GEAR_1)
        {
            return ShiftSafetyStatus::OutOfRange;
        }
        return ShiftSafetyStatus::Allowed;
    case SHIFT_REQUEST_NEUTRAL:
        if (gear_count == GEAR_1)
        {
            return ShiftSafetyStatus::Allowed;
        }
        return ShiftSafetyStatus::NeutralUnavailable;
    case SHIFT_REQUEST_NONE:
    default:
        return ShiftSafetyStatus::InvalidRequest;
    }
}

static uint16_t target_position_for_shift_request(ShiftRequest request, Gear gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        // The driver sees an N12345 pattern, but the real gearbox is 1N2345.
        // Virtual N->1 is a physical "half-shift" from neutral to 1st. Not exactly half tho; values set in config.hpp
        if (gear_count == GEAR_N)
        {
            return config::SHIFT_NEUTRAL_TO_1_STOP_POSITION;
        }
        return config::SHIFT_UP_STOP_POSITION;
    case SHIFT_REQUEST_DOWN:
        return config::SHIFT_DOWN_STOP_POSITION;
    case SHIFT_REQUEST_NEUTRAL:
        return config::SHIFT_1_TO_NEUTRAL_STOP_POSITION;
    case SHIFT_REQUEST_NONE:
    default:
        return 0;
    }
}

static float torque_for_shift_request(ShiftRequest request, Gear gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        // The driver sees an N12345 pattern, but the real gearbox is 1N2345.
        // Virtual N->1 is a physical half-shift from neutral to 1st, values set in config.hpp.
        if (gear_count == GEAR_N)
        {
            return config::SHIFT_NEUTRAL_TO_1_TORQUE;
        }
        return config::SHIFT_UP_TORQUE;
    case SHIFT_REQUEST_DOWN:
        return config::SHIFT_DOWN_TORQUE;
    case SHIFT_REQUEST_NEUTRAL:
        return config::SHIFT_1_TO_NEUTRAL_TORQUE;
    case SHIFT_REQUEST_NONE:
    default:
        return 0.0f;
    }
}

static bool read_encoder_before_shift(ShiftRequest request, Gear gear_count, AMT20 &encoder)
{
    uint16_t position = 0;
    if (encoder.read_position(position))
    {
        ESP_LOGI(TAG_SHIFTER,
                 "Encoder pre-check passed: request=%s gear=%s position=%u",
                 shift_request_to_string(request),
                 gear_to_string(gear_count),
                 static_cast<unsigned>(position));
        return true;
    }

    ESP_LOGE(TAG_SHIFTER,
             "Encoder pre-check failed: request=%s gear=%s",
             shift_request_to_string(request),
             gear_to_string(gear_count));
    return false;
}

static ShiftPositionStatus shifted_position_status(ShiftRequest request, Gear gear_count, AMT20 &encoder)
{
    uint16_t position = 0;
    if (!encoder.read_position(position))
    {
        ESP_LOGE(TAG_SHIFTER,
                 "Encoder read failed while checking shift position: request=%s gear=%s",
                 shift_request_to_string(request),
                 gear_to_string(gear_count));
        return ShiftPositionStatus::ReadFault;
    }

    ESP_LOGI(TAG_SHIFTER,
             "Encoder position: phase=shift request=%s position=%u",
             shift_request_to_string(request),
             static_cast<unsigned>(position));

    const uint16_t target = target_position_for_shift_request(request, gear_count);
    if (AMT20::position_is_near(position, target, config::SHIFT_POSITION_TOLERANCE))
    {
        ESP_LOGI(TAG_SHIFTER,
                 "Shift target reached: request=%s position=%u target=%u tolerance=%u",
                 shift_request_to_string(request),
                 static_cast<unsigned>(position),
                 static_cast<unsigned>(target),
                 static_cast<unsigned>(config::SHIFT_POSITION_TOLERANCE));
        return ShiftPositionStatus::Reached;
    }

    return ShiftPositionStatus::NotReached;
}

static Gear gear_after_successful_shift(ShiftRequest request, Gear gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        if (gear_count == GEAR_N)
        {
            return GEAR_1;
        }
        if (gear_count < GEAR_5)
        {
            return static_cast<Gear>(gear_count + 1);
        }
        return gear_count;
    case SHIFT_REQUEST_DOWN:
        if (gear_count > GEAR_1)
        {
            return static_cast<Gear>(gear_count - 1);
        }
        return gear_count;
    case SHIFT_REQUEST_NEUTRAL:
        return GEAR_N;
    case SHIFT_REQUEST_NONE:
    default:
        return gear_count;
    }
}

static int8_t gear_to_can_value(Gear gear)
{
    return static_cast<int8_t>(gear);
}

static void publish_shifter_can_status(Gear gear_count,
                                       ShifterCanStatus status,
                                       ShiftRequest last_request)
{
    set_shifter_task_can_status({
        gear_to_can_value(gear_count),
        static_cast<uint8_t>(status),
        static_cast<uint8_t>(last_request),
    });
}

static void sync_gear_count_from_can(Gear &gear_count)
{
    if (config::STANDALONE_TESTING)
    {
        return;
    }

    const CanTaskOutput can_output = get_can_task_output();
    if (can_output.switches_valid && can_output.neutral_pos)
    {
        if (gear_count != GEAR_N)
        {
            ESP_LOGI(TAG_SHIFTER,
                     "Synced gear from neutral sensor: previous=%s gear=N",
                     gear_to_string(gear_count));
        }
        gear_count = GEAR_N;
        return;
    }

    if (!can_output.speed_gear_valid || !can_output.is_moving)
    {
        return;
    }

    Gear ecu_gear = gear_count;
    if (ecu_gear_to_gear(can_output.ecu_gear, ecu_gear))
    {
        if (gear_count != ecu_gear)
        {
            ESP_LOGI(TAG_SHIFTER,
                     "Synced gear from ECU: previous=%s ecu_gear=%s",
                     gear_to_string(gear_count),
                     gear_to_string(ecu_gear));
        }
        gear_count = ecu_gear;
    }
    else
    {
        ESP_LOGW(TAG_SHIFTER,
                 "Ignoring invalid ECU gear: ecu_gear=%d",
                 static_cast<int>(can_output.ecu_gear));
    }
}

static bool can_safety_data_available()
{
    if (config::STANDALONE_TESTING)
    {
        return true;
    }

    const CanTaskOutput can_output = get_can_task_output();
    return can_output.speed_gear_valid && can_output.switches_valid;
}

static void process_shift_request(ShiftRequest request, Gear &gear_count, AMT20 &encoder, ESC &esc)
{
    ESP_LOGI(TAG_SHIFTER, "Shift request accepted: %s", shift_request_to_string(request));

    sync_gear_count_from_can(gear_count);
    publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_SHIFTING, request);

    ShiftSafetyStatus safety_status = can_safety_data_available()
                                          ? shift_request_safety_status(request, gear_count)
                                          : ShiftSafetyStatus::CanUnavailable;
    if (safety_status == ShiftSafetyStatus::Allowed &&
        !read_encoder_before_shift(request, gear_count, encoder))
    {
        safety_status = ShiftSafetyStatus::EncoderFault;
    }

    if (safety_status != ShiftSafetyStatus::Allowed)
    {
        ESP_LOGW(TAG_SHIFTER,
                 "Shift rejected: request=%s gear=%s reason=%s",
                 shift_request_to_string(request),
                 gear_to_string(gear_count),
                 shift_safety_status_to_string(safety_status));
        esc.set_torque(0.0f);
        publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_REJECTED, request);
        ESP_LOGI(TAG_SHIFTER, "ESC neutral command sent: shift rejected");
        return;
    }

    const float shift_torque = torque_for_shift_request(request, gear_count);
    const int shift_torque_milli = static_cast<int>(shift_torque * 1000.0f);
    ESP_LOGI(TAG_SHIFTER,
             "Commanding ESC: request=%s gear=%s torque_milli=%d target=%u timeout_us=%lld",
             shift_request_to_string(request),
             gear_to_string(gear_count),
             shift_torque_milli,
             static_cast<unsigned>(target_position_for_shift_request(request, gear_count)),
             static_cast<long long>(config::SHIFT_TIMEOUT_US));
    esc.set_torque(shift_torque);

    bool shifted = false;
    bool encoder_fault = false;
    const int64_t shift_start_us = esp_timer_get_time();

    // Core actuation window. Do not block/yield here.
    while ((esp_timer_get_time() - shift_start_us) < config::SHIFT_TIMEOUT_US)
    {
        const ShiftPositionStatus position_status = shifted_position_status(request, gear_count, encoder);
        if (position_status == ShiftPositionStatus::Reached)
        {
            shifted = true;
            break;
        }
        if (position_status == ShiftPositionStatus::ReadFault)
        {
            encoder_fault = true;
            break;
        }
    }

    esc.set_torque(0.0f);
    ESP_LOGI(TAG_SHIFTER, "ESC neutral command sent: shift window ended");
    const int64_t shift_duration_us = esp_timer_get_time() - shift_start_us;
    ESP_LOGI(TAG_SHIFTER,
             "ESC command cleared: request=%s duration_us=%lld shifted=%d encoder_fault=%d",
             shift_request_to_string(request),
             static_cast<long long>(shift_duration_us),
             static_cast<int>(shifted),
             static_cast<int>(encoder_fault));

    if (shifted)
    {
        gear_count = gear_after_successful_shift(request, gear_count);
        ESP_LOGI(TAG_SHIFTER,
                 "Internal gear count updated after successful shift: gear=%s",
                 gear_to_string(gear_count));
        publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_SHIFT_COMPLETE, request);
    }
    else if (encoder_fault)
    {
        ESP_LOGE(TAG_SHIFTER,
                 "Shift aborted due to encoder read fault: %s",
                 shift_request_to_string(request));
        publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_ENCODER_FAULT, request);
    }
    else
    {
        ESP_LOGE(TAG_SHIFTER, "Shift timed out: %s", shift_request_to_string(request));
        publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_SHIFT_TIMEOUT, request);
    }
}

void shifter_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG_SHIFTER, "Shifter task started");

    AMT20 encoder(config::ENCODER_SPI_HOST,
                  config::ENCODER_CS_GPIO,
                  config::ENCODER_MOSI_GPIO,
                  config::ENCODER_MISO_GPIO,
                  config::ENCODER_SCLK_GPIO);
    ESC esc(config::ESC_PWM_GPIO);

    // Internal software gear count. On startup this should always be neutral.
    Gear gear_count = GEAR_N;
    publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_IDLE, SHIFT_REQUEST_NONE);

    // Constructor already sends neutral, but be explicit at startup.
    esc.set_torque(0.0f);
    ESP_LOGI(TAG_SHIFTER, "ESC neutral command sent: startup");

    esp_err_t err = setup_shift_inputs(xTaskGetCurrentTaskHandle());
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_SHIFTER, "Shift input setup failed: %s", esp_err_to_name(err));
        esc.set_torque(0.0f);
        publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_INPUT_FAULT, SHIFT_REQUEST_NONE);
        ESP_LOGI(TAG_SHIFTER, "ESC neutral command sent: shift input setup failed");
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    for (;;)
    {
        // Wake periodically to print encoder position, or immediately when an
        // input ISR latches a shift request.
        if (ulTaskNotifyTake(pdTRUE, config::ENCODER_IDLE_LOG_INTERVAL) == 0)
        {
            sync_gear_count_from_can(gear_count);
            publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_IDLE, SHIFT_REQUEST_NONE);

            uint16_t position = 0;
            if (encoder.read_position(position))
            {
                ESP_LOGI(TAG_SHIFTER,
                         "Encoder position: phase=idle position=%u",
                         static_cast<unsigned>(position));
            }
            else
            {
                ESP_LOGW(TAG_SHIFTER, "Encoder read failed: phase=idle");
            }
            continue;
        }

        // Take a local copy of the request and release the ISR-side latch.
        const ShiftRequest request = consume_pending_shift_request();
        if (request == SHIFT_REQUEST_NONE)
        {
            ESP_LOGW(TAG_SHIFTER, "Task woke without a pending shift request");
            publish_shifter_can_status(gear_count, SHIFTER_CAN_STATUS_IDLE, SHIFT_REQUEST_NONE);
            clear_and_enable_shift_inputs();
            continue;
        }

        process_shift_request(request, gear_count, encoder, esc);
        clear_and_enable_shift_inputs();
    }
}
