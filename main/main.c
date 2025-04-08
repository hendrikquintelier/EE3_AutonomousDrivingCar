#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "wifi_logger.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <math.h>
#include <float.h>
#include "motorcontrol.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "surroundings.h"
#include "exploration_algorithm/direction.h"



drive_result_t last_drive_result;



// ============================================================================
// Main Entry Point
// ============================================================================
void app_main(void)
{
    // Initialize WiFi
    printf("Initializing WiFi...\n");
    if (wifi_logger_init() != ESP_OK)
    {
        printf("Failed to initialize WiFi logger\n");
        return;
    }
    printf("WiFi initialization complete\n");

    // Initialize hardware
    printf("Initializing MPU...\n");
    if (mpu_init() != 0)
    {
        printf("Failed to initialize MPU\n");
        return;
    }
    printf("MPU initialization complete\n");

    printf("Initializing ultrasonic sensors...\n");
    ultrasonic_init();
    printf("Ultrasonic initialization complete\n");

    printf("Initializing motors...\n");
    motor_init();
    printf("Motor initialization complete\n");

    printf("Initializing encoders...\n");
    encoder_init();
    printf("Encoder initialization complete\n");

    // Wait for MPU to stabilize
    printf("Waiting for MPU to stabilize...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("MPU stabilization complete\n");

    log_remote("System initialization complete");

    

    log_remote("Starting forward drive test (40 cm)");
    
    // Drive forward 40 cm with no heading or distance compensation
    drive_result_t result = motor_forward_distance(0.0f, 0.0f);
    
    log_remote("Drive test complete. Final heading error: %.2f°, Distance overshoot: %.2f cm",
               result.heading, result.forward_distance_overshoot);
    
    
}
