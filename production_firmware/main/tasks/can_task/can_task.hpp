#pragma once

#include <cstdint>

struct CanTaskOutput
{
    bool is_moving;
    int8_t ecu_gear;
    bool neutral_pos;
    bool speed_gear_valid;
    bool switches_valid;
};

enum ShifterCanStatus : uint8_t
{
    SHIFTER_CAN_STATUS_STARTUP = 0,
    SHIFTER_CAN_STATUS_IDLE = 1,
    SHIFTER_CAN_STATUS_SHIFTING = 2,
    SHIFTER_CAN_STATUS_REJECTED = 3,
    SHIFTER_CAN_STATUS_SHIFT_COMPLETE = 4,
    SHIFTER_CAN_STATUS_SHIFT_TIMEOUT = 5,
    SHIFTER_CAN_STATUS_ENCODER_FAULT = 6,
    SHIFTER_CAN_STATUS_INPUT_FAULT = 7,
};

struct ShifterTaskCanStatus
{
    int8_t internal_gear;
    uint8_t shifter_status;
    uint8_t last_request;
};

CanTaskOutput get_can_task_output();
void set_can_task_output(const CanTaskOutput &output);

ShifterTaskCanStatus get_shifter_task_can_status();
void set_shifter_task_can_status(const ShifterTaskCanStatus &status);

void can_task(void *arg);
