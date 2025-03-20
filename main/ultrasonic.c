#include "ultrasonic.h"

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin assignments (adjust to your wiring)
#define TRIG_LEFT GPIO_NUM_40
#define ECHO_LEFT GPIO_NUM_39
#define TRIG_FRONT GPIO_NUM_42
#define ECHO_FRONT GPIO_NUM_41
#define TRIG_RIGHT GPIO_NUM_1
#define ECHO_RIGHT GPIO_NUM_2

// Max wait time for echo (µs) ~30ms
static const int max_wait_us = 30000;

static void init_ultrasonic_sensor(gpio_num_t trig_pin, gpio_num_t echo_pin);
static float measure_distance(gpio_num_t trig_pin, gpio_num_t echo_pin);

void ultrasonic_init(void)
{
    init_ultrasonic_sensor(TRIG_LEFT, ECHO_LEFT);
    init_ultrasonic_sensor(TRIG_FRONT, ECHO_FRONT);
    init_ultrasonic_sensor(TRIG_RIGHT, ECHO_RIGHT);
}

ultrasonic_readings_t ultrasonic_get_all(void)
{
    ultrasonic_readings_t readings;
    readings.left = measure_distance(TRIG_LEFT, ECHO_LEFT);
    readings.front = measure_distance(TRIG_FRONT, ECHO_FRONT);
    readings.right = measure_distance(TRIG_RIGHT, ECHO_RIGHT);
    return readings;
}

static void init_ultrasonic_sensor(gpio_num_t trig_pin, gpio_num_t echo_pin)
{
    gpio_reset_pin(trig_pin);
    gpio_reset_pin(echo_pin);

    gpio_set_direction(trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(trig_pin, 0); // default LOW

    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
}

static float measure_distance(gpio_num_t trig_pin, gpio_num_t echo_pin)
{
    int wait_time = 0;
    uint64_t start_time = 0, end_time = 0;

    // 1. Send 10µs pulse
    gpio_set_level(trig_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(0.01)); // 10µs
    gpio_set_level(trig_pin, 0);

    // 2. Wait for ECHO to go HIGH
    wait_time = 0;
    while (gpio_get_level(echo_pin) == 0 && wait_time < max_wait_us)
    {
        vTaskDelay(pdMS_TO_TICKS(0.001)); // 1µs
        wait_time++;
    }
    if (wait_time >= max_wait_us)
    {
        return -1.0f;
    }
    start_time = esp_timer_get_time();

    // 3. Wait for ECHO to go LOW
    wait_time = 0;
    while (gpio_get_level(echo_pin) == 1 && wait_time < max_wait_us)
    {
        vTaskDelay(pdMS_TO_TICKS(0.001)); // 1µs
        wait_time++;
    }
    if (wait_time >= max_wait_us)
    {
        return -1.0f;
    }
    end_time = esp_timer_get_time();

    // 4. Calculate distance (speed of sound ~0.0343 cm/µs)
    float duration_us = (float)(end_time - start_time);
    float distance_cm = (duration_us * 0.0343f) / 2.0f;
    return distance_cm;
}
