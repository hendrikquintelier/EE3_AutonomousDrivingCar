#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_wifi_types.h"
#include "esp_wifi_default.h"
#include "esp_mac.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include "esp_timer.h"
#include "driver/gpio.h"

// WiFi credentials
#define WIFI_SSID "Boenks"
#define WIFI_PASSWORD "boenkie123"

// Server configuration
#define SERVER_IP "192.168.190.87"
#define SERVER_PORT 1234

#define MAX_BUFFER_SIZE 50
#define MAX_ERROR_TIME_MS 10000 // 10 seconds
#define MAX_MESSAGE_LENGTH 160
#define MAX_WIFI_RETRY_COUNT 40  // Increased from 20 to 40 attempts
#define WIFI_RETRY_DELAY_MS 2000 // Increased from 1000 to 2000ms

typedef struct
{
    char message[MAX_MESSAGE_LENGTH];
    uint32_t timestamp;
} buffered_message_t;

static int sock = -1;
static struct sockaddr dest_addr;

static TaskHandle_t wifi_logger_task_handle = NULL;
static EventGroupHandle_t wifi_event_group;
static char server_ip[16];
static uint16_t server_port;
#define WIFI_CONNECTED_BIT BIT0
#define MAX_RETRY_COUNT 3
#define RETRY_DELAY_MS 1000
#define MIN_LOG_INTERVAL_MS 200
static uint32_t last_log_time = 0;
static bool wifi_connected = false;


// Message buffer
static buffered_message_t message_buffer[MAX_BUFFER_SIZE];
static int buffer_head = 0;
static int buffer_tail = 0;
static int buffer_count = 0;
static uint32_t first_error_time = 0;
static bool error_reported = false;

// Socket state variables
static bool socket_initialized = false;
static bool socket_needs_recreation = false;
static uint32_t last_socket_recreation = 0;
static uint32_t last_successful_send = 0;
static int retry_count = 0;
static uint32_t last_retry_time = 0;
#define SOCKET_RECREATION_COOLDOWN_MS 5000 // 5 seconds cooldown between recreation attempts
#define SOCKET_TIMEOUT_MS 10000            // 10 seconds without successful send before recreation

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi disconnected! Attempting to reconnect...\n");
        wifi_connected = false;
        socket_initialized = false; // Reset socket state on disconnect
        first_error_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        // Try to reconnect
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("WiFi connected! IP address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        error_reported = false;
        first_error_time = 0;

        // Only mark for recreation if this is a new IP address
        if (sock >= 0)
        {
            esp_netif_ip_info_t current_ip;
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif != NULL && esp_netif_get_ip_info(netif, &current_ip) == ESP_OK)
            {
                if (current_ip.ip.addr != event->ip_info.ip.addr)
                {
                    socket_needs_recreation = true;
                }
            }
        }
    }
}

// Task function for WiFi logger
static void wifi_logger_task(void *pvParameters)
{
    while (1)
    {
        // Give other tasks a chance to run
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}





// Function to create and configure UDP socket
static esp_err_t create_udp_socket(const char *server_ip, uint16_t server_port)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Close existing socket if any
    if (sock >= 0)
    {
        close(sock);
        sock = -1;
    }

    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        printf("ERROR: Unable to create socket: errno %d\n", errno);
        return ESP_FAIL;
    }

    // Set up destination address
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_addr.s_addr = inet_addr(server_ip);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(server_port);
    memcpy(&dest_addr, &addr, sizeof(struct sockaddr));

    // Print connection details
    printf("Attempting to connect to %s:%d\n", server_ip, server_port);

    // Get and print local IP address
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {
        printf("Local IP: " IPSTR "\n", IP2STR(&ip_info.ip));
        printf("Local Gateway: " IPSTR "\n", IP2STR(&ip_info.gw));
        printf("Local Netmask: " IPSTR "\n", IP2STR(&ip_info.netmask));
    }

    // Set socket options with minimal buffer sizes
    int broadcast = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        printf("WARNING: Failed to set broadcast option: errno %d\n", errno);
    }

    // Try to set minimal socket buffer sizes, but don't fail if it doesn't work
    int send_buf = 32;
    if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_buf, sizeof(send_buf)) < 0)
    {
        if (errno != 109) // Ignore "Operation not supported" error
        {
            printf("WARNING: Failed to set send buffer size: errno %d\n", errno);
        }
    }

    int recv_buf = 32;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buf, sizeof(recv_buf)) < 0)
    {
        if (errno != 109) // Ignore "Operation not supported" error
        {
            printf("WARNING: Failed to set receive buffer size: errno %d\n", errno);
        }
    }

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        printf("WARNING: Failed to set socket timeout: errno %d\n", errno);
    }

    // Send test packet
    const char *test_msg = "ESP32 Connected";
    printf("Sending test packet to %s:%d...\n", server_ip, server_port);
    int err = sendto(sock, test_msg, strlen(test_msg), 0, &dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        printf("WARNING: Failed to send test packet: errno %d\n", errno);
        printf("Error details: %s\n", strerror(errno));
        close(sock);
        sock = -1;
        return ESP_FAIL;
    }
    else
    {
        printf("Successfully sent test packet to %s:%d\n", server_ip, server_port);
        last_successful_send = current_time;
        socket_initialized = true;
    }

    last_socket_recreation = current_time;
    return ESP_OK;
}

// Initialize WiFi connection
static esp_err_t init_wifi(const char *ssid, const char *password)
{
    printf("init_wifi called with SSID: '%s', Password length: %d\n", WIFI_SSID, strlen(WIFI_PASSWORD));

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

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    // Copy SSID and password to the configuration structure
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    printf("WiFi config set - SSID: '%s', Password length: %d\n", wifi_config.sta.ssid, strlen((char *)wifi_config.sta.password));

    // Set WiFi mode and start
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    return ESP_OK;
}

esp_err_t wifi_logger_init(void)
{
    printf("Initializing WiFi logger...\n");
    printf("Using WiFi credentials - SSID: '%s', Password length: %d\n", WIFI_SSID, strlen(WIFI_PASSWORD));
    printf("Connecting to server at %s:%d\n", SERVER_IP, SERVER_PORT);

    // Store server info
    strncpy(server_ip, SERVER_IP, sizeof(server_ip) - 1);
    server_ip[sizeof(server_ip) - 1] = '\0';
    server_port = SERVER_PORT;

    // Create event group for WiFi events
    wifi_event_group = xEventGroupCreate();

    // Initialize WiFi
    if (init_wifi(WIFI_SSID, WIFI_PASSWORD) != ESP_OK)
    {
        printf("Failed to initialize WiFi\n");
        return ESP_FAIL;
    }

    // Wait for WiFi to be connected and have a valid IP
    wifi_ap_record_t ap_info;
    int retry_count = 0;
    bool got_valid_ip = false;

    printf("Attempting to connect to WiFi network '%s'...\n", WIFI_SSID);

    while (retry_count < MAX_WIFI_RETRY_COUNT && !got_valid_ip)
    {
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            printf("WiFi connected to %s\n", ap_info.ssid);
            printf("Signal strength: %d dBm\n", ap_info.rssi);
            printf("Channel: %d\n", ap_info.primary);
            printf("Authentication mode: %d\n", ap_info.authmode);

            // Get and print IP info
            esp_netif_ip_info_t ip_info;
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
            {
                // Check if we have a valid IP (not 0.0.0.0)
                if (ip_info.ip.addr != 0)
                {
                    printf("Got valid IP: " IPSTR "\n", IP2STR(&ip_info.ip));
                    printf("Gateway: " IPSTR "\n", IP2STR(&ip_info.gw));
                    printf("Netmask: " IPSTR "\n", IP2STR(&ip_info.netmask));
                    got_valid_ip = true;
                    wifi_connected = true;
                    break;
                }
                else
                {
                    printf("Waiting for valid IP address...\n");
                }
            }
        }
        printf("Waiting for WiFi connection... (attempt %d/%d)\n", retry_count + 1, MAX_WIFI_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
        retry_count++;
    }

    if (!got_valid_ip)
    {
        printf("ERROR: Failed to get valid IP address after %d attempts\n", MAX_WIFI_RETRY_COUNT);
        return ESP_FAIL;
    }

    // Add a longer delay to ensure network stack is ready
    printf("Waiting for network stack to stabilize...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Create UDP socket
    if (create_udp_socket(SERVER_IP, SERVER_PORT) != ESP_OK)
    {
        return ESP_FAIL;
    }
    printf("UDP socket created successfully\n");

    // Create WiFi logger task
    BaseType_t xReturned = xTaskCreate(
        wifi_logger_task,
        "wifi_logger",
        2048,
        NULL,
        5,
        &wifi_logger_task_handle);

    if (xReturned != pdPASS)
    {
        printf("ERROR: Failed to create WiFi logger task\n");
        close(sock);
        sock = -1;
        return ESP_FAIL;
    }

    printf("WiFi logger initialized successfully - connecting to %s:%d\n", SERVER_IP, SERVER_PORT);
    return ESP_OK;
}


void log_remote(const char *format, ...)
{
    // Only log if WiFi is connected and socket is initialized
    if (!wifi_connected || !socket_initialized)
    {
        printf("Warning: Attempted to log before WiFi connection was established\n");
        return;
    }

    va_list args;
    va_start(args, format);
    char message[256];
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    // Add timestamp
    char log_message[512];
    uint64_t timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds
    snprintf(log_message, sizeof(log_message), "[%llu] %s", timestamp, message);

    // Send to server
    sendto(sock, log_message, strlen(log_message), 0, &dest_addr, sizeof(dest_addr));
}

void wifi_logger_deinit(void)
{
    printf("Shutting down WiFi logger...\n");

    // Delete the WiFi logger task
    if (wifi_logger_task_handle != NULL)
    {
        vTaskDelete(wifi_logger_task_handle);
        wifi_logger_task_handle = NULL;
    }

    if (sock >= 0)
    {
        close(sock);
        sock = -1;
        printf("Socket closed\n");
    }

    // Reset all state variables
    socket_initialized = false;
    socket_needs_recreation = false;
    retry_count = 0;
    last_retry_time = 0;
    last_successful_send = 0;
    last_socket_recreation = 0;

    // Delete event group
    if (wifi_event_group != NULL)
    {
        vEventGroupDelete(wifi_event_group);
        wifi_event_group = NULL;
    }

    printf("WiFi logger shutdown complete\n");
}