/**
Gear Shifter Code 
For any questions contact James Platt

The design philosophy prioritises safety. At the time of writing this software my feeling was the best way of doing 
this was for the shifting events (i.e UP, DOWN, NEUTRAL) to have their own dedicated fully blocking ISRs.
In these ISRs any additional interrupts would be IGNORED and freertos task switching would be DISABLED.
It would have completely "locked" the autoshifter into the shift event until either:
    - The "shifted" position is reached (i.e succesful shift has occured)
    - A timeout occurs. This is logged as an error, but not treated as a hard fail.

I did not end up doing it this way because the shift itself is a relatively long mechanical event, and ISRs are not
really the right place to sit and wait for mechanical things to happen. The bigger problem is
timing/watchdog/driver behaviour: a long ISR can upset software timers, watchdog
servicing, and any ESP-IDF/FreeRTOS driver code that expects the RTOS and interrupts to still be alive.

The approach used here is therefore slightly different. The shifting events are still detected by their own dedicated
GPIO ISRs, but these ISRs do NOT actually do the shift. They only latch the first shift request, disable any more shift
input interrupts, and wake the high priority shifter task.

The shifter task then effectively "owns" the autoshifter until either:
    - The "shifted" position is reached (i.e succesful shift has occured)
    - A timeout occurs. This is logged as an error, but not treated as a hard fail.

During this shift window any additional shift inputs are IGNORED because the shift GPIO interrupts are disabled.
The shifter task is also created at the highest priority and the core actuation window is written to be
non-blocking, so lower priority tasks should not randomly run in the middle of a shift.

When a shift event is detected (i.e user button input) a few safety checks are done before the ESC is commanded.
These are written to be as fast as possible without compromising on safety. Then the output PWM command to the ESC
is sent.

Some of the important safety checks are: 
    - the requested gear shift does not result in an out-of-range gear (i.e shifting into "6th")
    - the encoder can be read before the ESC is commanded
    
**/





//Esp logging library (a nicer way of "printing to serial" it allows you to specify different levels of importance e.g LOGW (warning) LOGE (error) LOGI (info)
#include "esp_log.h"

//freertos libraries for tasks
#include "freertos/FreeRTOS.h"                           
#include "freertos/task.h" 

#include "config.hpp"
#include "can_task.hpp"
#include "shifter_task.hpp"

//Logging is seperated by tags
static const char *TAG_SYS = "SYSTEM";

// Entry point
extern "C" void app_main(void)
{
    ESP_LOGI(TAG_SYS, "Creating tasks");

    BaseType_t ok;
    bool success = true;

    //highest priority.
    ok = xTaskCreatePinnedToCore(shifter_task,
                                 "shifter_task",
                                 config::SHIFTER_TASK_STACK_WORDS,
                                 nullptr,
                                 config::SHIFTER_TASK_PRIORITY,
                                 nullptr,
                                 config::CONTROL_CORE);
    if(ok != pdPASS){success = false;}

    ok = xTaskCreatePinnedToCore(can_task,
                                 "can_task",
                                 config::CAN_TASK_STACK_WORDS,
                                 nullptr,
                                 config::CAN_TASK_PRIORITY,
                                 nullptr,
                                 config::CONTROL_CORE);
    if(ok != pdPASS){success = false;}
    
    // If task creation fails, the controller is in an unknown/partial startup
    // state. Stop interrupts and park forever instead of running without one of
    // the safety-critical tasks. (Hard fail)

    if(!success){
        ESP_LOGE(TAG_SYS, "Task creation failed!");
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    // app_main() has done its job once the tasks are alive. The tasks themselves
    // contain the arduino-style superloops that keep the application alive.
    ESP_LOGI(TAG_SYS, "Tasks created succesfully!");

    // Delete app_main task.
    vTaskDelete(NULL);
}
