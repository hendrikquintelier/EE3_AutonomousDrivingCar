#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "encoder.h"
#include "motorcontrol.h"
#include "ultrasonic.h"
#include "wifi_logger.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"

#define OBSTACLE_DISTANCE_THRESHOLD 30.0f // 30cm threshold for obstacle detection
#define BRAKE_START_DISTANCE 20.0f        // Start braking at 20cm
#define BRAKE_STOP_DISTANCE 5.0f          // Stop at 5cm
#define MIN_SPEED 0.1f                    // Minimum speed before stopping

// WiFi configuration
#define WIFI_SSID "Boenks"
#define WIFI_PASSWORD "boenkie123"
#define SERVER_IP "192.168.1.4" // Updated to match laptop's IP address
#define SERVER_PORT 1234

// UART configuration
#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 1
#define UART_RX_PIN 3

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

        // Configure TCP/IP settings
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
        printf("IP: " IPSTR ", GW: " IPSTR ", NETMASK: " IPSTR "\n",
               IP2STR(&ip_info.ip), IP2STR(&ip_info.gw), IP2STR(&ip_info.netmask));
    }
}

// Function to scan for available networks
static void scan_wifi_networks(void)
{
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true};

    printf("Scanning for WiFi networks...\n");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    if (ap_count == 0)
    {
        printf("No networks found!\n");
        return;
    }

    wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) * ap_count);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));

    printf("Found %d networks:\n", ap_count);
    for (int i = 0; i < ap_count; i++)
    {
        printf("%d. SSID: %s, RSSI: %d dBm\n", i + 1, ap_records[i].ssid, ap_records[i].rssi);
    }

    free(ap_records);
}

void app_main(void)
{
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122, // Add flow control threshold
        .source_clk = UART_SCLK_DEFAULT};

    // Install UART driver with larger buffer
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Add a delay after UART initialization
    vTaskDelay(pdMS_TO_TICKS(100));

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

    // Get the default netif
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL)
    {
        printf("Failed to get netif\n");
        return;
    }

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Add a delay after WiFi initialization
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
            .listen_interval = 3,
            .threshold.rssi = -127, // Accept any signal strength
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH},
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for WiFi to start

    // Now scan for networks
    scan_wifi_networks();

    // Wait for WiFi to connect with timeout
    printf("Waiting for WiFi to connect...\n");
    int retry_count = 0;
    while (retry_count < 20)
    { // 20 seconds timeout
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            printf("WiFi connected to SSID: %s\n", ap_info.ssid);
            printf("WiFi signal strength: %d dBm\n", ap_info.rssi);
            printf("Authentication mode: %d\n", ap_info.authmode);
            break;
        }

        // Check if we're still trying to connect
        wifi_mode_t mode;
        if (esp_wifi_get_mode(&mode) == ESP_OK)
        {
            printf("WiFi mode: %d\n", mode);
        }

        // Get connection status
        wifi_sta_list_t sta_list;
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK)
        {
            printf("Connected stations: %d\n", sta_list.num);
        }

        // Try to reconnect if disconnected
        if (retry_count % 5 == 0)
        {
            printf("Attempting to reconnect...\n");
            ESP_ERROR_CHECK(esp_wifi_disconnect());
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_ERROR_CHECK(esp_wifi_connect());
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
    }
    if (retry_count >= 20)
    {
        printf("WiFi connection timeout!\n");
        return; // Exit if WiFi connection fails
    }

    // Now initialize WiFi logger
    wifi_logger_init(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT);
    my_wifi_log("Starting initialization sequence...\n");

    // 1. Initialize all components
    my_wifi_log("Initializing MPU...\n");
    mpu_init();

    my_wifi_log("Initializing encoders...\n");
    encoder_init();

    my_wifi_log("Initializing motors...\n");
    motor_init();

    my_wifi_log("Initializing ultrasonic sensors...\n");
    ultrasonic_init();

    // 2. Set initial heading
    my_wifi_log("Setting initial heading to 0 degrees...\n");
    motor_set_desired_heading(0.0f);

    my_wifi_log("Starting obstacle detection test...\n");
    my_wifi_log("Driving forward at 60%% speed...\n");

    // Test motor movement first
    my_wifi_log("Testing motor movement...\n");
    motor_drive_straight(0.6f);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Run for 1 second
    motor_brake();
    my_wifi_log("Motor test complete.\n");

    // Drive forward until obstacle detected
    while (1)
    {
        // Get ultrasonic readings
        ultrasonic_readings_t readings = ultrasonic_get_all();
        my_wifi_log("Front distance: %.1f cm\n", readings.front);

        // Progressive braking logic
        if (readings.front < BRAKE_START_DISTANCE)
        {
            // Calculate speed based on distance
            float speed_factor = (readings.front - BRAKE_STOP_DISTANCE) / (BRAKE_START_DISTANCE - BRAKE_STOP_DISTANCE);

            // Clamp speed factor between 0 and 1
            if (speed_factor < 0)
                speed_factor = 0;
            if (speed_factor > 1)
                speed_factor = 1;

            // Calculate target speed (between MIN_SPEED and 0.6)
            float target_speed = MIN_SPEED + (speed_factor * (0.6f - MIN_SPEED));

            my_wifi_log("Progressive braking: distance=%.1f cm, speed_factor=%.2f, target_speed=%.2f\n",
                        readings.front, speed_factor, target_speed);

            // If we're very close to the obstacle, stop completely
            if (readings.front <= BRAKE_STOP_DISTANCE)
            {
                my_wifi_log("Reached stop distance (%.1f cm)! Initiating full brake...\n", readings.front);
                motor_brake();
                my_wifi_log("Car stopped.\n");
                break;
            }

            // Drive at the calculated speed
            motor_drive_straight(target_speed);
        }
        else
        {
            // Normal driving speed when far from obstacle
            motor_drive_straight(0.6f);
        }

        // Small delay to prevent overwhelming the ultrasonic sensor
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Keep the program running
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
