#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "wifi_logger.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float left;
        float front;
        float right;
    } ultrasonic_readings_t;

    /**
     * @brief Initialize ultrasonic pins for left, front, right.
     */
    void ultrasonic_init(void);

    /**
     * @brief Returns all three distances in a single call.
     * @return ultrasonic_readings_t containing left, front, and right distances in centimeters
     */
    ultrasonic_readings_t ultrasonic_get_all(void);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_H
