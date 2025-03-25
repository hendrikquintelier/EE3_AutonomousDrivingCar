#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdarg.h>
#include "esp_err.h"

    // Initialize WiFi and start logging
    esp_err_t wifi_logger_init(const char *ssid, const char *password, const char *server_ip, uint16_t server_port);

    // Send a formatted log message
    void my_wifi_log(const char *format, ...);

    // Close WiFi connection
    void wifi_logger_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_LOGGER_H