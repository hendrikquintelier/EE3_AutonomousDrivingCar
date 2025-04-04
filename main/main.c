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

#define MOTOR_SPEED 0.7f // 70% speed

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

    // Start driving forward with yaw control
    log_remote("Starting movement sequence...");

    // First turn 90 degrees right
    log_remote("Performing 90-degree right turn...");

    // Then drive forward
    // log_remote("Starting straight drive test...");
    motor_forward_distance(MOTOR_SPEED, 40.0f);

    log_remote("Test complete!");
}
