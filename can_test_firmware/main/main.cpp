#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
