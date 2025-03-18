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

#define MAX_BUFFER_SIZE 50
#define MAX_ERROR_TIME_MS 10000 // 10 seconds
#define MAX_MESSAGE_LENGTH 160

typedef struct
{
    char message[MAX_MESSAGE_LENGTH];
    uint32_t timestamp;
} buffered_message_t;

static int sock = -1;
static struct sockaddr dest_addr;
static char rx_buffer[128];
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
static char wifi_ssid[32];
static char wifi_password[64];

// Message buffer
static buffered_message_t message_buffer[MAX_BUFFER_SIZE];
static int buffer_head = 0;
static int buffer_tail = 0;
static int buffer_count = 0;
static uint32_t first_error_time = 0;
static bool error_reported = false;

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi disconnected! Attempting to reconnect...\n");
        wifi_connected = false;
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

// Function to add message to buffer
static bool add_to_buffer(const char *message, uint32_t timestamp)
{
    if (buffer_count >= MAX_BUFFER_SIZE)
    {
        return false; // Buffer full
    }

    strncpy(message_buffer[buffer_tail].message, message, MAX_MESSAGE_LENGTH - 1);
    message_buffer[buffer_tail].message[MAX_MESSAGE_LENGTH - 1] = '\0';
    message_buffer[buffer_tail].timestamp = timestamp;

    buffer_tail = (buffer_tail + 1) % MAX_BUFFER_SIZE;
    buffer_count++;
    return true;
}

// Function to send buffered messages
static void send_buffered_messages(void)
{
    while (buffer_count > 0 && wifi_connected)
    {
        // Send the oldest message
        int err = sendto(sock, message_buffer[buffer_head].message,
                         strlen(message_buffer[buffer_head].message), 0,
                         &dest_addr, sizeof(dest_addr));

        if (err < 0)
        {
            if (errno == 118)
            { // Connection timeout
                // Put the message back in the buffer
                return;
            }
            break;
        }

        // Remove the sent message from buffer
        buffer_head = (buffer_head + 1) % MAX_BUFFER_SIZE;
        buffer_count--;

        // Small delay between messages
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Function to create and configure UDP socket
static esp_err_t create_udp_socket(const char *server_ip, uint16_t server_port)
{
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
    }

    // Set socket options with minimal buffer sizes
    int broadcast = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        printf("WARNING: Failed to set broadcast option: errno %d\n", errno);
    }

    // Set minimal socket buffer sizes
    int send_buf = 32; // Further reduced buffer size
    if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_buf, sizeof(send_buf)) < 0)
    {
        printf("WARNING: Failed to set send buffer size: errno %d\n", errno);
    }

    int recv_buf = 32; // Further reduced buffer size
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buf, sizeof(recv_buf)) < 0)
    {
        printf("WARNING: Failed to set receive buffer size: errno %d\n", errno);
    }

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        printf("WARNING: Failed to set socket timeout: errno %d\n", errno);
    }

    return ESP_OK;
}

esp_err_t wifi_logger_init(const char *ssid, const char *password, const char *server_ip_param, uint16_t server_port_param)
{
    printf("Initializing WiFi logger...\n");

    // Store WiFi credentials
    strncpy(wifi_ssid, ssid, sizeof(wifi_ssid) - 1);
    wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
    strncpy(wifi_password, password, sizeof(wifi_password) - 1);
    wifi_password[sizeof(wifi_password) - 1] = '\0';

    // Store server info
    strncpy(server_ip, server_ip_param, sizeof(server_ip) - 1);
    server_ip[sizeof(server_ip) - 1] = '\0';
    server_port = server_port_param;

    // Create event group for WiFi events
    wifi_event_group = xEventGroupCreate();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Wait for WiFi to be connected
    wifi_ap_record_t ap_info;
    int retry_count = 0;
    while (retry_count < 20)
    {
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            printf("WiFi connected to %s\n", ap_info.ssid);
            wifi_connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
    }
    if (retry_count >= 20)
    {
        printf("ERROR: WiFi not connected\n");
        return ESP_FAIL;
    }

    // Create UDP socket
    if (create_udp_socket(server_ip, server_port) != ESP_OK)
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

    printf("WiFi logger initialized successfully - connecting to %s:%d\n", server_ip, server_port);
    return ESP_OK;
}

// Function to format timestamp
static void format_timestamp(char *timestamp, size_t max_len, uint32_t ms_since_boot)
{
    uint32_t total_seconds = ms_since_boot / 1000;
    uint32_t ms = ms_since_boot % 1000;
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;

    snprintf(timestamp, max_len, "[%02lu:%02lu:%02lu.%03lu]",
             hours, minutes, seconds, ms);
}

void my_wifi_log(const char *format, ...)
{
    static int retry_count = 0;
    static uint32_t last_retry_time = 0;
    static uint32_t last_successful_send = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Check if WiFi is connected
    if (!wifi_connected)
    {
        // Try to reconnect if not connected
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
        {
            printf("WiFi disconnected, attempting to reconnect...\n");
            esp_wifi_connect();
            return;
        }
        else
        {
            printf("WiFi reconnected successfully\n");
            wifi_connected = true;
        }
    }

    // Rate limit log messages
    if (current_time - last_log_time < MIN_LOG_INTERVAL_MS)
    {
        return;
    }
    last_log_time = current_time;

    // If socket is invalid or we haven't had a successful send in a while, try to recreate it
    if (sock < 0 || (current_time - last_successful_send > 5000)) // 5 seconds without successful send
    {
        if (current_time - last_retry_time >= RETRY_DELAY_MS && retry_count < MAX_RETRY_COUNT)
        {
            printf("Attempting to recreate socket (attempt %d/%d)...\n", retry_count + 1, MAX_RETRY_COUNT);
            if (create_udp_socket(server_ip, server_port) == ESP_OK)
            {
                retry_count = 0;
                printf("Socket recreated successfully\n");
                // Try to send buffered messages after socket recreation
                send_buffered_messages();
            }
            else
            {
                retry_count++;
                last_retry_time = current_time;
            }
        }
        return;
    }

    va_list args;
    va_start(args, format);
    vsnprintf(rx_buffer, sizeof(rx_buffer), format, args);
    va_end(args);

    // Add formatted timestamp to the message
    char timestamp[32];
    format_timestamp(timestamp, sizeof(timestamp), current_time);

    // Combine timestamp and message
    char full_message[160];
    snprintf(full_message, sizeof(full_message), "%s %s", timestamp, rx_buffer);

    // Try to send the message with a small delay between attempts
    int err = sendto(sock, full_message, strlen(full_message), 0,
                     &dest_addr, sizeof(dest_addr));

    if (err < 0)
    {
        printf("Failed to send message: errno %d\n", errno);

        // Only buffer message if it's not an out of memory error
        if (errno != 12) // 12 is ENOMEM (Out of memory)
        {
            add_to_buffer(full_message, current_time);
        }

        // Check if we should report an error
        if (!error_reported && first_error_time > 0)
        {
            uint32_t error_duration = current_time - first_error_time;
            if (error_duration >= MAX_ERROR_TIME_MS)
            {
                printf("Sustained connection issues for %d seconds\n", MAX_ERROR_TIME_MS / 1000);
                error_reported = true;
            }
        }

        // If we get an out of memory error, try to recover
        if (errno == 12)
        {
            printf("Out of memory error, attempting to recover...\n");
            // Close and recreate socket
            if (sock >= 0)
            {
                close(sock);
                sock = -1;
            }
            // Clear message buffer
            buffer_head = 0;
            buffer_tail = 0;
            buffer_count = 0;
            // Try to recreate socket
            create_udp_socket(server_ip, server_port);
        }
    }
    else
    {
        retry_count = 0;
        last_successful_send = current_time;
        // Try to send any buffered messages after successful send
        send_buffered_messages();
    }

    // Add a small delay between sends to prevent overwhelming the system
    vTaskDelay(pdMS_TO_TICKS(20)); // Increased delay between sends
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

    // Delete event group
    if (wifi_event_group != NULL)
    {
        vEventGroupDelete(wifi_event_group);
        wifi_event_group = NULL;
    }

    printf("WiFi logger shutdown complete\n");
}