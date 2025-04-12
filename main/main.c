#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "wifi_logger.h"
#include "motorcontrol.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "nvs_flash.h"

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize components
    // log_remote("Initializing components...");
    motor_init();
    ultrasonic_init();
    mpu_init();
    encoder_init();
    // log_remote("All components initialized successfully");

    // Wait for MPU to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Main loop
    while (1) {
        // Monitor MPU interrupts
        mpu_monitor_interrupts();
        
        // Run test sequence
        // log_remote("Starting test sequence...");
        test_turning();
        // log_remote("Test sequence completed");
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent overwhelming the system
    }
}
