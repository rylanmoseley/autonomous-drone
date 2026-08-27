#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "control_system/control_system.h"

extern "C" void app_main(void)
{
    printf("Flight Controller Initializing...\n");
    
    // TODO: Instantiate physical ESP32 IMU and Motor drivers here
    // e.g. ESP32_IMU imu(...);
    // e.g. ESP32_Motors motors(...);
    
    // ControlSystem drone(&imu, &motors);
    // drone.init();
    
    while(1) {
        // drone.tick();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
