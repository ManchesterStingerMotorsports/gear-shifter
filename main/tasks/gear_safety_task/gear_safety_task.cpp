#include "gear_safety_task.hpp"
#include "gear_state.hpp"
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
// that the requested shift has completed.
static const int64_t SHIFT_TIMEOUT_US = 200000;

// Maximum accepted age for the ECU gear packet used by the pre-shift safety
// checks.
static const uint32_t ECU_GEAR_MAX_AGE_MS = 200;

// Placeholder torque commands for each shift type. These are normalised values
// passed into ESC::set_torque(), not real measured torque.
static const float SHIFT_UP_TORQUE = 0.25f;
static const float SHIFT_DOWN_TORQUE = -0.25f;
static const float SHIFT_NEUTRAL_FROM_1_TORQUE = 0.25f;
static const float SHIFT_NEUTRAL_FROM_2_TORQUE = -0.25f;

// Final calibration values to fill in from measured encoder positions at the
// mechanical stop/shifted positions.
static const uint16_t SHIFT_UP_STOP_POSITION = 0;
static const uint16_t SHIFT_DOWN_STOP_POSITION = 0;
static const uint16_t SHIFT_NEUTRAL_FROM_1_STOP_POSITION = 0;
static const uint16_t SHIFT_NEUTRAL_FROM_2_STOP_POSITION = 0;
static const uint16_t SHIFT_POSITION_TOLERANCE = 20;

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

static float torque_for_shift_request(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return SHIFT_UP_TORQUE;
    case SHIFT_REQUEST_DOWN:
        return SHIFT_DOWN_TORQUE;
    case SHIFT_REQUEST_NEUTRAL:
        if (gear_count == GEAR_1)
        {
            return SHIFT_NEUTRAL_FROM_1_TORQUE;
        }
        if (gear_count == GEAR_2)
        {
            return SHIFT_NEUTRAL_FROM_2_TORQUE;
        }
        return 0.0f;
    case SHIFT_REQUEST_NONE:
    default:
        return 0.0f;
    }
}

// SAFETY CHECK: shift is in range
static bool requested_shift_is_in_range(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return gear_count < GEAR_5;
    case SHIFT_REQUEST_DOWN:
        return gear_count > GEAR_1;
    case SHIFT_REQUEST_NEUTRAL:
        return gear_count == GEAR_1 || gear_count == GEAR_2;
    case SHIFT_REQUEST_NONE:
    default:
        return false;
    }
}

static uint16_t target_position_for_shift_request(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return SHIFT_UP_STOP_POSITION;
    case SHIFT_REQUEST_DOWN:
        return SHIFT_DOWN_STOP_POSITION;
    case SHIFT_REQUEST_NEUTRAL:
        if (gear_count == GEAR_1)
        {
            return SHIFT_NEUTRAL_FROM_1_STOP_POSITION;
        }
        if (gear_count == GEAR_2)
        {
            return SHIFT_NEUTRAL_FROM_2_STOP_POSITION;
        }
        return 0;
    case SHIFT_REQUEST_NONE:
    default:
        return 0;
    }
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
                 "Encoder read failed while checking shift position: request=%s gear=%d",
                 shift_request_to_string(request),
                 static_cast<int>(gear_count));
        return ShiftPositionStatus::ReadFault;
    }

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


static void report_rejected_shift(ShiftRequest request)
{
    ESP_LOGW(TAG_GEAR_SAFETY, "Shift rejected: %s", shift_request_to_string(request));
}

static bool gear_count_matches_fresh_ecu_snapshot(const GearSnapshot &ecu_gear,
                                                  bool ecu_gear_is_stale,
                                                  gear_t gear_count)
{
    return !ecu_gear_is_stale && ecu_gear.gear == gear_count;
}

static bool resync_gear_count_from_ecu_if_mismatched(const GearSnapshot &ecu_gear,
                                                     bool ecu_gear_is_stale,
                                                     gear_t &gear_count)
{
    if (ecu_gear_is_stale || ecu_gear.gear == gear_count)
    {
        return false;
    }

    const gear_t old_gear_count = gear_count;
    ESP_LOGW(TAG_GEAR_SAFETY,
             "Internal gear count resynced from ECU: internal=%d ecu=%d",
             static_cast<int>(old_gear_count),
             static_cast<int>(ecu_gear.gear));

    gear_count = ecu_gear.gear;
    return true;
}

// Update the internal gear count after a confirmed successful shift. This is
// intentionally separate from the actuation code so the rule is easy to audit.
static gear_t gear_after_successful_shift(ShiftRequest request, gear_t gear_count)
{
    switch (request)
    {
    case SHIFT_REQUEST_UP:
        return static_cast<gear_t>(gear_count + 1);
    case SHIFT_REQUEST_DOWN:
        return static_cast<gear_t>(gear_count - 1);
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

    esc.set_torque(0.0f); //constructor already sets torque to 0 but its best to be explicit

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
        // Sleep until a shift input ISR latches a request.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Take a local copy of the request and release the ISR-side latch.
        const ShiftRequest request = consume_pending_shift_request();
        if (request == SHIFT_REQUEST_NONE)
        {
            ESP_LOGW(TAG_GEAR_SAFETY, "Task woke without a pending shift request");
            clear_and_enable_shift_inputs();
            continue;
        }

        ESP_LOGI(TAG_GEAR_SAFETY, "Shift request accepted: %s", shift_request_to_string(request));

        // Snapshot the ECU gear before the ESC is commanded.
        const GearSnapshot ecu_gear = read_ecu_gear_snapshot();
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        const bool ecu_gear_is_stale =
            ecu_gear_snapshot_is_stale(ecu_gear, now_ms, ECU_GEAR_MAX_AGE_MS);

        const bool shift_is_in_range = requested_shift_is_in_range(request, gear_count);
        const bool ecu_matches_internal_count =
            gear_count_matches_fresh_ecu_snapshot(ecu_gear, ecu_gear_is_stale, gear_count);

        uint16_t encoder_position = 0;
        const bool encoder_read_ok = encoder.read_position(encoder_position);
        const bool precheck_passed = shift_is_in_range && ecu_matches_internal_count && encoder_read_ok;

        ESP_LOGI(TAG_GEAR_SAFETY,
                 "Precheck: request=%s internal=%d ecu=%d ecu_fresh=%d in_range=%d encoder_ok=%d encoder=%u",
                 shift_request_to_string(request),
                 static_cast<int>(gear_count),
                 static_cast<int>(ecu_gear.gear),
                 static_cast<int>(!ecu_gear_is_stale),
                 static_cast<int>(shift_is_in_range),
                 static_cast<int>(encoder_read_ok),
                 static_cast<unsigned>(encoder_position));

        if (!precheck_passed)
        {
            const gear_t failed_internal_gear_count = gear_count;
            const bool resynced_gear_count =
                resync_gear_count_from_ecu_if_mismatched(ecu_gear, ecu_gear_is_stale, gear_count);

            esc.set_torque(0.0f);
            ESP_LOGW(TAG_GEAR_SAFETY,
                     "Precheck failed: in_range=%d ecu_fresh=%d ecu=%d internal=%d encoder_ok=%d resynced=%d",
                     static_cast<int>(shift_is_in_range),
                     static_cast<int>(!ecu_gear_is_stale),
                     static_cast<int>(ecu_gear.gear),
                     static_cast<int>(failed_internal_gear_count),
                     static_cast<int>(encoder_read_ok),
                     static_cast<int>(resynced_gear_count));
            report_rejected_shift(request);
            clear_and_enable_shift_inputs();
            continue;
        }

        const float shift_torque = torque_for_shift_request(request, gear_count);
        const int shift_torque_milli = static_cast<int>(shift_torque * 1000.0f);
        ESP_LOGI(TAG_GEAR_SAFETY,
                 "Commanding ESC: request=%s gear=%d torque_milli=%d target=%u timeout_us=%lld",
                 shift_request_to_string(request),
                 static_cast<int>(gear_count),
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

        esc.set_torque(0.0f);
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
                     "Internal gear count updated after successful shift: gear=%d",
                     static_cast<int>(gear_count));
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
