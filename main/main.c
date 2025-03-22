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

// WiFi configuration
#define WIFI_SSID "Boenks"
#define WIFI_PASSWORD "boenkie123"
#define SERVER_IP "192.168.118.87"
#define SERVER_PORT 1234

// Event handler for WiFi events
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        printf("WiFi connecting to SSID: %s\n", WIFI_SSID);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi disconnected!\n");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("WiFi connected! IP address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP adapter
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Initialize WiFi logger
    wifi_logger_init(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT);
    printf("Starting MPU yaw logging...\n");

    // Initialize MPU
    printf("Initializing MPU...\n");
    mpu_init();

    // Wait for MPU to stabilize
    printf("Waiting for MPU to stabilize...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Main loop - log MPU yaw readings
    float last_yaw = 0;
    uint32_t last_print_time = 0;
    const uint32_t PRINT_INTERVAL_MS = 100; // Print every 100ms

    while (1)
    {
        mpu_data_t orientation = mpu_get_orientation();
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Only print if enough time has passed
        if (current_time - last_print_time >= PRINT_INTERVAL_MS)
        {
            // Calculate yaw change rate
            float yaw_change = fabsf(orientation.yaw - last_yaw);
            if (yaw_change > 180.0f) // Handle wrap-around
            {
                yaw_change = 360.0f - yaw_change;
            }

            // Log to both serial console and WiFi logger
            printf("Yaw: %.2f° (Change: %.2f°/s)\n",
                   orientation.yaw,
                   yaw_change * (1000.0f / PRINT_INTERVAL_MS));

            char log_message[100];
            snprintf(log_message, sizeof(log_message), "Yaw: %.2f° (Change: %.2f°/s)",
                     orientation.yaw,
                     yaw_change * (1000.0f / PRINT_INTERVAL_MS));
            my_wifi_log(log_message);

            last_yaw = orientation.yaw;
            last_print_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent overwhelming the system
    }
}
