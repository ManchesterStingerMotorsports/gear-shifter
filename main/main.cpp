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
really the right place to sit and wait for mechanical things to happen. Holding a hard interrupt context for the whole
shift would probably stop CAN updates, but that is not really the main problem. I am okay with CAN being stale during
the actual shift window because the important ECU gear comparison should already have been snapshotted before the ESC
is commanded. The bigger problem is timing/watchdog/driver behaviour: a long ISR can upset software timers, watchdog
servicing, and any ESP-IDF/FreeRTOS driver code that expects the RTOS and interrupts to still be alive.

The approach used here is therefore slightly different. The shifting events are still detected by their own dedicated
GPIO ISRs, but these ISRs do NOT actually do the shift. They only latch the first shift request, disable any more shift
input interrupts, and wake the high priority gear safety task.

The gear safety task then effectively "owns" the autoshifter until either:
    - The "shifted" position is reached (i.e succesful shift has occured)
    - A timeout occurs. This is logged as an error, but not treated as a hard fail.

During this shift window any additional shift inputs are IGNORED because the shift GPIO interrupts are disabled.
The gear safety task is also created at the highest priority and the core actuation window is written to be
non-blocking, so lower priority tasks should not randomly run in the middle of a shift.

When a shift event is detected (i.e user button input) a few safety checks are done before the ESC is commanded.
These are written to be as fast as possible without compromising on safety. Then the output PWM command to the ESC
is sent.

Some of the important safety checks are: 
    - the requested gear shift does not result in an out-of-range gear (i.e shifting into "6th")
    - the ECU measured gear and the gear shifter's internal count "agree"
    
**/





//Esp logging library (a nicer way of "printing to serial" it allows you to specify different levels of importance e.g LOGW (warning) LOGE (error) LOGI (info)
#include "esp_log.h"

//freertos libraries for tasks
#include "freertos/FreeRTOS.h"                           
#include "freertos/task.h" 

//the two tasks that are explicitly being used (technically there are 4 as MSM_CAN creates two more)
#include "can_task.hpp"
#include "gear_safety_task.hpp"

//Logging is seperated by tags
static const char *TAG_SYS = "SYSTEM";

// Entry point
extern "C" void app_main(void)
{
    ESP_LOGI(TAG_SYS, "Creating tasks");

    BaseType_t ok;
    bool success = true;
    
    // CAN is deliberately lower priority than the shifter. It should keep the
    // latest ECU/CAN state fresh, but it must not preempt the core shift action.
    ok = xTaskCreate(can_task, "can_task", 4096, nullptr, 1, nullptr);
    if(ok != pdPASS){success = false;}

    //highest priority.
    ok = xTaskCreate(gear_safety_task, "gear_safety_task", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr);
    if(ok != pdPASS){success = false;}
    
    // If task creation fails, the controller is in an unknown/partial startup
    // state. Stop interrupts and park forever instead of running without one of
    // the safety-critical tasks. Hard fail.

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
