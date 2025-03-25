#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#include <esp_err.h>

// Initialize WiFi logger
esp_err_t wifi_logger_init(void);

// Log a message remotely
void log_remote(const char *format, ...);

#endif // WIFI_LOGGER_H