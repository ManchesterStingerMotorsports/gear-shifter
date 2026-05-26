#include "gear_safety_task.hpp"
#include "shift_inputs.hpp"

#include "AMT20.hpp"
#include "ESC.hpp"

#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"


static const char *TAG_GEAR_SAFETY = "GEAR_SAFETY_TASK";

static const gpio_num_t ESC_PWM_GPIO = GPIO_NUM_9;

static const gpio_num_t ENCODER_CS_GPIO = GPIO_NUM_10;
static const gpio_num_t ENCODER_MOSI_GPIO = GPIO_NUM_11;
static const gpio_num_t ENCODER_SCLK_GPIO = GPIO_NUM_12;
static const gpio_num_t ENCODER_MISO_GPIO = GPIO_NUM_13;

static AMT20 encoder(SPI2_HOST,
                     ENCODER_CS_GPIO,
                     ENCODER_MOSI_GPIO,
                     ENCODER_MISO_GPIO,
                     ENCODER_SCLK_GPIO);

// Maximum time the shift task will drive the ESC while waiting for confirmation
// that the requested shift has completed. This may need to be adjusted.
static const int64_t SHIFT_TIMEOUT_US = 200000;

// Placeholder torque commands for each shift type. These are normalised values
// passed into ESC::set_torque(). These may need to be modified during testing
static const float SHIFT_UP_TORQUE = 0.4f;
static const float SHIFT_DOWN_TORQUE = -0.4f;
static const float SHIFT_NEUTRAL_FROM_1_TORQUE = 0.4f;
static const float SHIFT_NEUTRAL_FROM_2_TORQUE = -0.4f;

// Final calibration values to fill in from measured encoder positions at the
// mechanical stop/shifted positions.
static const uint16_t BASE_POSITION = 2502;
static const uint16_t SHIFT_UP_STOP_POSITION = 2290;
static const uint16_t SHIFT_DOWN_STOP_POSITION = 2670;
static const uint16_t SHIFT_NEUTRAL_FROM_1_STOP_POSITION = BASE_POSITION;
static const uint16_t SHIFT_NEUTRAL_FROM_2_STOP_POSITION = BASE_POSITION;
static const uint16_t SHIFT_POSITION_TOLERANCE = 20;
static const TickType_t ENCODER_IDLE_LOG_INTERVAL = pdMS_TO_TICKS(100);

enum gear_t
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

static void print_encoder_position(const char *phase)
{
    uint16_t position = 0;
    if (encoder.read_position(position))
    {
        ESP_LOGI(TAG_GEAR_SAFETY,
                 "Encoder position: phase=%s position=%u",
                 phase,
                 static_cast<unsigned>(position));
    }
    else
    {
        ESP_LOGW(TAG_GEAR_SAFETY, "Encoder read failed: phase=%s", phase);
    }
}

static const char *gear_to_string(gear_t gear)
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
        return "neutral is only available from 1st or 2nd";
    case ShiftSafetyStatus::EncoderFault:
        return "encoder pre-check failed";
    default:
        return "unknown";
    }
}

static float torque_for_neutral_from_position(uint16_t position)
{
    if (AMT20::position_is_near(position, BASE_POSITION, SHIFT_POSITION_TOLERANCE))
    {
        return 0.0f;
    }

    return position > BASE_POSITION ? SHIFT_NEUTRAL_FROM_1_TORQUE : SHIFT_NEUTRAL_FROM_2_TORQUE;
}

static float torque_for_shift_request(ShiftRequest request, uint16_t current_position)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return SHIFT_UP_TORQUE;
    case SHIFT_REQUEST_DOWN:
        return SHIFT_DOWN_TORQUE;
    case SHIFT_REQUEST_NEUTRAL:
        return torque_for_neutral_from_position(current_position);
    case SHIFT_REQUEST_NONE:
    default:
        return 0.0f;
    }
}

static ShiftSafetyStatus shift_request_safety_status(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return gear_count == GEAR_5 ? ShiftSafetyStatus::OutOfRange : ShiftSafetyStatus::Allowed;
    case SHIFT_REQUEST_DOWN:
        return gear_count == GEAR_1 ? ShiftSafetyStatus::OutOfRange : ShiftSafetyStatus::Allowed;
    case SHIFT_REQUEST_NEUTRAL:
        if (gear_count == GEAR_1 || gear_count == GEAR_2)
        {
            return ShiftSafetyStatus::Allowed;
        }
        return ShiftSafetyStatus::NeutralUnavailable;
    case SHIFT_REQUEST_NONE:
    default:
        return ShiftSafetyStatus::InvalidRequest;
    }
}

//take shift request and return the target position that the encoder wants to reach
static uint16_t target_position_for_shift_request(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return SHIFT_UP_STOP_POSITION;
    case SHIFT_REQUEST_DOWN:
        return SHIFT_DOWN_STOP_POSITION;
    case SHIFT_REQUEST_NEUTRAL:
        return gear_count == GEAR_1 ? SHIFT_NEUTRAL_FROM_1_STOP_POSITION : SHIFT_NEUTRAL_FROM_2_STOP_POSITION;
    case SHIFT_REQUEST_NONE:
    default:
        return 0;
    }
}

static bool read_encoder_before_shift(ShiftRequest request, gear_t gear_count, uint16_t &position)
{
    if (encoder.read_position(position))
    {
        ESP_LOGI(TAG_GEAR_SAFETY,
                 "Encoder pre-check passed: request=%s gear=%s position=%u",
                 shift_request_to_string(request),
                 gear_to_string(gear_count),
                 static_cast<unsigned>(position));
        return true;
    }

    ESP_LOGE(TAG_GEAR_SAFETY,
             "Encoder pre-check failed: request=%s gear=%s",
             shift_request_to_string(request),
             gear_to_string(gear_count));
    return false;
}

// Polling hook used during the shift actuation window. It must be fast and
// non-blocking because the high-priority shift task spins on this until success
// or timeout.
static ShiftPositionStatus shifted_position_status(ShiftRequest request, gear_t gear_count)
{
    uint16_t position = 0;
    if (!encoder.read_position(position))
    {
        ESP_LOGE(TAG_GEAR_SAFETY,
                 "Encoder read failed while checking shift position: request=%s gear=%s",
                 shift_request_to_string(request),
                 gear_to_string(gear_count));
        return ShiftPositionStatus::ReadFault;
    }

    ESP_LOGI(TAG_GEAR_SAFETY,
             "Encoder position: phase=shift request=%s position=%u",
             shift_request_to_string(request),
             static_cast<unsigned>(position));

    const uint16_t target = target_position_for_shift_request(request, gear_count);
    if (AMT20::position_is_near(position, target, SHIFT_POSITION_TOLERANCE))
    {
        ESP_LOGI(TAG_GEAR_SAFETY,
                 "Shift target reached: request=%s position=%u target=%u tolerance=%u",
                 shift_request_to_string(request),
                 static_cast<unsigned>(position),
                 static_cast<unsigned>(target),
                 static_cast<unsigned>(SHIFT_POSITION_TOLERANCE));
        return ShiftPositionStatus::Reached;
    }

    return ShiftPositionStatus::NotReached;
}


static void report_shift_timeout(ShiftRequest request)
{
    ESP_LOGE(TAG_GEAR_SAFETY, "Shift timed out: %s", shift_request_to_string(request));
}


static void command_esc_neutral(ESC &esc, const char *reason)
{
    esc.set_torque(0.0f);
    ESP_LOGI(TAG_GEAR_SAFETY, "ESC neutral command sent: %s", reason);
}

// Update the internal gear count after a confirmed successful shift. This is
// intentionally separate from the actuation code so the rule is easy to audit.
static gear_t gear_after_successful_shift(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        if (gear_count == GEAR_N)
        {
            return GEAR_2;
        }
        if (gear_count < GEAR_5)
        {
            return static_cast<gear_t>(gear_count + 1);
        }
        return gear_count;
    case SHIFT_REQUEST_DOWN:
        if (gear_count == GEAR_N)
        {
            return GEAR_1;
        }
        if (gear_count > GEAR_1)
        {
            return static_cast<gear_t>(gear_count - 1);
        }
        return gear_count;
    case SHIFT_REQUEST_NEUTRAL:
        return GEAR_N;
    case SHIFT_REQUEST_NONE:
    default:
        return gear_count;
    }
}

void gear_safety_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG_GEAR_SAFETY, "Gear safety task started");

    // Constructing ESC
    ESC esc(ESC_PWM_GPIO);

    command_esc_neutral(esc, "startup"); //constructor already sets torque to 0 but its best to be explicit

    // Register the shift input GPIO ISRs
    esp_err_t err = setup_shift_inputs(xTaskGetCurrentTaskHandle());
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_GEAR_SAFETY, "Shift input setup failed: %s", esp_err_to_name(err));
        esc.set_torque(0.0f);
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    // Internal software gear count. On startup this should always be neutral.
    gear_t gear_count = GEAR_N;

    for (;;)
    {
        // Wake periodically to print encoder position, or immediately when an
        // input ISR latches a shift request.
        if (ulTaskNotifyTake(pdTRUE, ENCODER_IDLE_LOG_INTERVAL) == 0)
        {
            print_encoder_position("idle");
            continue;
        }

        // Take a local copy of the request and release the ISR-side latch.
        const ShiftRequest request = consume_pending_shift_request();
        if (request == SHIFT_REQUEST_NONE)
        {
            ESP_LOGW(TAG_GEAR_SAFETY, "Task woke without a pending shift request");
            clear_and_enable_shift_inputs();
            continue;
        }

        ESP_LOGI(TAG_GEAR_SAFETY, "Shift request accepted: %s", shift_request_to_string(request));

        ShiftSafetyStatus safety_status = shift_request_safety_status(request, gear_count);
        uint16_t current_position = 0;
        if (safety_status == ShiftSafetyStatus::Allowed &&
            !read_encoder_before_shift(request, gear_count, current_position))
        {
            safety_status = ShiftSafetyStatus::EncoderFault;
        }

        if (safety_status != ShiftSafetyStatus::Allowed)
        {
            ESP_LOGW(TAG_GEAR_SAFETY,
                     "Shift rejected: request=%s gear=%s reason=%s",
                     shift_request_to_string(request),
                     gear_to_string(gear_count),
                     shift_safety_status_to_string(safety_status));
            command_esc_neutral(esc, "shift rejected");
            clear_and_enable_shift_inputs();
            continue;
        }

        const float shift_torque = torque_for_shift_request(request, current_position);
        const int shift_torque_milli = static_cast<int>(shift_torque * 1000.0f);
        ESP_LOGI(TAG_GEAR_SAFETY,
                 "Commanding ESC: request=%s gear=%s torque_milli=%d target=%u timeout_us=%lld",
                 shift_request_to_string(request),
                 gear_to_string(gear_count),
                 shift_torque_milli,
                 static_cast<unsigned>(target_position_for_shift_request(request, gear_count)),
                 static_cast<long long>(SHIFT_TIMEOUT_US));
        esc.set_torque(shift_torque);

        bool shifted = false;
        bool encoder_fault = false;
        const int64_t shift_start_us = esp_timer_get_time();

        // Core actuation window. Do not block/yield here.
        while ((esp_timer_get_time() - shift_start_us) < SHIFT_TIMEOUT_US)
        {
            const ShiftPositionStatus position_status = shifted_position_status(request, gear_count);
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

        command_esc_neutral(esc, "shift window ended");
        const int64_t shift_duration_us = esp_timer_get_time() - shift_start_us;
        ESP_LOGI(TAG_GEAR_SAFETY,
                 "ESC command cleared: request=%s duration_us=%lld shifted=%d encoder_fault=%d",
                 shift_request_to_string(request),
                 static_cast<long long>(shift_duration_us),
                 static_cast<int>(shifted),
                 static_cast<int>(encoder_fault));

        if (shifted)
        {
            gear_count = gear_after_successful_shift(request, gear_count);
            ESP_LOGI(TAG_GEAR_SAFETY,
                     "Internal gear count updated after successful shift: gear=%s",
                     gear_to_string(gear_count));
        }
        else if (encoder_fault)
        {
            ESP_LOGE(TAG_GEAR_SAFETY, "Shift aborted due to encoder read fault: %s",
                     shift_request_to_string(request));
        }
        else
        {
            report_shift_timeout(request);
        }

        clear_and_enable_shift_inputs();
    }
}
