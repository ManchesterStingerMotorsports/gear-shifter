#pragma once

#include <cstdint>

struct CanTaskOutput
{
    bool is_moving;
    int8_t ecu_gear;
    bool neutral_pos;
};

CanTaskOutput get_can_task_output();
void set_can_task_output(const CanTaskOutput &output);

void can_task(void *arg);
