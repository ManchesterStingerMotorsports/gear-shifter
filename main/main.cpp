/**
Gear Shifter Code 
For any questions contact James Platt

The design philosophy prioritises safety. At the time of writing this software my feeling was the best way of doing 
this was for the shifting events (i.e UP, DOWN, NEUTRAL) to have their own dedicated fully blocking ISRs.
In these ISRs any additional interrupts are IGNORED and freertos task switching is DISABLED. 
It effectively "locks" the autoshifter into a "critical section" of code which it will only exit if either:
    - The "shifted" position is reached (i.e succesful shift has occured)
    - A timeout occurs. This will be logged as an error on CAN. (but not hard fail).
When a shift event is detected (i.e user button input) a few safety checks are done. These are written to be as fast 
as possible without compromising on safety. Then the output PWM command to the ESC is sent. 

Some of the important safety checks are: 
    - the requested gear shift does not result in an out-of-range gear (i.e shifting into "6th")
    - the ECU measured gear and the gear shifter's internal count "agree"
    
**/





//Esp logging library (a nicer way of "printing to serial" it allows you to specify different levels of importance e.g LOGW (warning) LOGE (error) LOGI (info)
#include "esp_log.h"

//freertos libraries for tasks and mutexes 
#include "freertos/FreeRTOS.h"                           
#include "freertos/semphr.h"
#include "freertos/task.h" 

//the two tasks that are being used.
 
#include "can_task.hpp"
#include "gear_safety_task.hpp"

//logging tag 
static const char *TAG_SYS = "SYSTEM";

//entry point for the program
extern "C" void app_main(void)
{
    ESP_LOGI(TAG_SYS, "Hello World");
    ESP_LOGI(TAG_SYS, "Creating mutexes...");
       
    SemaphoreHandle_t gear_count_mutex = xSemaphoreCreateMutex(); 
    SemaphoreHandle_t gear_ecu_mutex = xSemaphoreCreateMutex();   

    if((gear_count_mutex == NULL) || gear_ecu_mutex == NULL){
        ESP_LOGE(TAG_SYS, "Mutex creation failed!");
        for(;;); //die
    }

    ESP_LOGI(TAG_SYS, "Mutexes created succesfully!");

    ESP_LOGI(TAG_SYS, "Creating tasks");


    BaseType_t ok;
    bool success = true;
    
    ok = xTaskCreate(can_task, "can_task", 4096, nullptr, 1, nullptr);
    if(ok != pdPASS){success = false;}
    
    ok = xTaskCreate(gear_safety_task, "gear_safety_task", 4096, nullptr, 2, nullptr);
    if(ok != pdPASS){success = false;}
    
    if(!success){
        ESP_LOGE(TAG_SYS, "Task creation failed!");
        taskDISABLE_INTERRUPTS();
        for (;;);
    }

    ESP_LOGI(TAG_SYS, "Tasks created succesfully!");
    ESP_LOGI(TAG_SYS, "Goodbye World");

    vTaskDelete(NULL);
}
