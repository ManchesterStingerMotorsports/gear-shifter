#pragma once

#include <stdint.h>

// Shared gear representation 
enum gear_t
{
    GEAR_N = 0,
    GEAR_1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,
};

struct GearSnapshot
{
    gear_t gear;
    uint32_t timestamp_ms;
    bool valid;     // False until at least one valid ECU gear packet has been decoded.
};

// Called by the CAN side after decoding a valid ECU gear packet.
void update_ecu_gear_snapshot(gear_t gear, uint32_t timestamp_ms);

// Called by the gear safety task immediately before the pre-shift checks.
GearSnapshot read_ecu_gear_snapshot();

// Helper for rejecting stale ECU gear data. 
bool ecu_gear_snapshot_is_stale(const GearSnapshot &snapshot,
                                uint32_t now_ms,
                                uint32_t max_age_ms);
