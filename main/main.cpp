#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "can_task.hpp"
#include "gear_safety_task.hpp"

static const char *TAG_SYS = "SYSTEM";

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
