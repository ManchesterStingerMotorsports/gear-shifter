#include "can_task.hpp"

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "MSM_CAN.hpp"

static const char *TAG = "CAN_TASK";

void can_task(void *arg)
{
    (void)arg;
    MSM_CAN::set_hardware_filters();            //for now TX only 
    MSM_CAN::init();
    

    
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
