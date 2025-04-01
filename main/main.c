#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "wifi_logger.h"
#include "ultrasonic.h"
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

#define MOTOR_SPEED 0.7f // 70% speed

// ============================================================================
// Main Entry Point
// ============================================================================
void app_main(void)
{
    // Initialize components
    wifi_logger_init();
    mpu_init();
    ultrasonic_init();
    motor_init();

    // Run motor test sequence
    // test_motors_sequence();

    // Wait for MPU to stabilize
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Test sequence
    // log_remote("Starting test sequence...");

    // First turn left
    log_remote("Testing left turn...");
    motor_turn_90(false);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a second between turns

    // Then turn right
    log_remote("Testing right turn...");
    motor_turn_90(true);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a second between turns

    // Finally drive forward until wall detection
    log_remote("Starting forward drive until wall detection...");
    motor_forward_constant_speed(0.7f); // Drive at 50% speed

    // Check for available paths and turn if found

    // log_remote("Checking for available paths...");
    // if (check_and_turn_available_path())
    //{
    //     log_remote("Path found and turned, continuing forward...");
    //     motor_forward_constant_speed(0.5f); // Continue driving after turn
    // }
    // else
    //{
    //     log_remote("No available paths found");
    // }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a second after stopping

    log_remote("Test sequence complete!");
}
