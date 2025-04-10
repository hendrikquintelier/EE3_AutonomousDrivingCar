#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "wifi_logger.h"
#include "motorcontrol.h"
#include "ultrasonic.h"
#include "encoder.h"



void app_main(void)
{
    // Initialize components
    log_remote("Initializing components...");
    wifi_logger_init();
    mpu_init();
    ultrasonic_init();
    motor_init();
    encoder_init();
    log_remote("All components initialized");
    
    // Wait for everything to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Run the navigation test
    test_navigation();
    
    // End of test
    log_remote("Test sequence complete");
}
