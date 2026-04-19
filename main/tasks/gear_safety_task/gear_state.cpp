#include "gear_state.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// Spinlock protecting the ECU gear snapshot.
static portMUX_TYPE ecu_gear_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

// The latest ECU gear state decoded from CAN. It starts invalid because the
// shifter should not trust ECU gear until at least one packet has arrived.
static GearSnapshot ecu_gear_snapshot = {
    GEAR_N,
    0,
    false,
};

void update_ecu_gear_snapshot(gear_t gear, uint32_t timestamp_ms)
{
    // The CAN task updates all fields under one short critical section so the
    // gear safety task can never read a half-old, half-new snapshot.
    portENTER_CRITICAL(&ecu_gear_snapshot_mux);
    ecu_gear_snapshot.gear = gear;
    ecu_gear_snapshot.timestamp_ms = timestamp_ms;
    ecu_gear_snapshot.valid = true;
    portEXIT_CRITICAL(&ecu_gear_snapshot_mux);
}

GearSnapshot read_ecu_gear_snapshot()
{
    GearSnapshot snapshot;

    // Copy the complete struct while protected, then return the local copy.
    // it can then be inspected without holding lock.
    portENTER_CRITICAL(&ecu_gear_snapshot_mux);
    snapshot = ecu_gear_snapshot;
    portEXIT_CRITICAL(&ecu_gear_snapshot_mux);

    return snapshot;
}

bool ecu_gear_snapshot_is_stale(const GearSnapshot &snapshot,
                                uint32_t now_ms,
                                uint32_t max_age_ms)
{
    // Invalid data is treated as stale.
    if (!snapshot.valid)
    {
        return true;
    }

    // Unsigned subtraction handles normal millisecond counter wraparound.
    return static_cast<uint32_t>(now_ms - snapshot.timestamp_ms) > max_age_ms;
}
