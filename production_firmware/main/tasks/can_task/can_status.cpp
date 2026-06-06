#include "can_status.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE can_task_output_mux = portMUX_INITIALIZER_UNLOCKED;

static CanTaskOutput can_task_output = {false, 0, false};

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
