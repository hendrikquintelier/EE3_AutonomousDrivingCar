#include "ultrasonic.h"
#include "wifi_logger.h" // For log_remote
#include <string.h>      // For strcpy
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

// Timing constants
#define TRIG_PULSE_US 10       // 10µs trigger pulse
#define MAX_WAIT_US 30000      // 30ms max wait time
#define SPEED_OF_SOUND 0.0343f // cm/µs at 20°C
#define MIN_DISTANCE 2.0f      // Minimum valid distance (cm)
#define MAX_DISTANCE 400.0f    // Maximum valid distance (cm)
#define MIN_ECHO_TIME 100      // Minimum echo time in microseconds (2cm)
#define MAX_ECHO_TIME 23270    // Maximum echo time in microseconds (400cm)
#define NUM_SAMPLES 3          // Number of samples to average

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

    // Configure echo pin with pull-down
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << echo_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}

static void delay_us(uint32_t us)
{
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us)
    {
        // Busy wait for precise timing
    }
}

static float measure_distance(gpio_num_t trig_pin, gpio_num_t echo_pin)
{
    float sum_distance = 0.0f;
    int valid_samples = 0;
    char sensor_name[10];

    // Set sensor name for logging
    if (trig_pin == TRIG_FRONT)
        strcpy(sensor_name, "Front");
    else if (trig_pin == TRIG_LEFT)
        strcpy(sensor_name, "Left");
    else
        strcpy(sensor_name, "Right");

    // Take multiple samples and average them
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        // Add delay between measurements
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms delay between measurements

        uint64_t start_time = 0, end_time = 0;
        uint32_t wait_time = 0;

        // 1. Send 10µs pulse
        gpio_set_level(trig_pin, 1);
        delay_us(TRIG_PULSE_US);
        gpio_set_level(trig_pin, 0);

        // 2. Wait for ECHO to go HIGH with timeout
        wait_time = 0;
        while (gpio_get_level(echo_pin) == 0 && wait_time < MAX_WAIT_US)
        {
            delay_us(1);
            wait_time++;
        }
        if (wait_time >= MAX_WAIT_US)
        {
            log_remote("%s sensor: No echo received (timeout waiting for HIGH) - Check wiring and power", sensor_name);
            continue; // Skip this sample
        }
        start_time = esp_timer_get_time();

        // 3. Wait for ECHO to go LOW with timeout
        wait_time = 0;
        while (gpio_get_level(echo_pin) == 1 && wait_time < MAX_WAIT_US)
        {
            delay_us(1);
            wait_time++;
        }
        if (wait_time >= MAX_WAIT_US)
        {
            log_remote("%s sensor: Echo never went LOW (timeout) - Check wiring and power", sensor_name);
            continue; // Skip this sample
        }
        end_time = esp_timer_get_time();

        // 4. Calculate duration and validate timing
        float duration_us = (float)(end_time - start_time);

        // Log raw timing data
        log_remote("%s sensor: Echo duration = %.1f µs, Start time = %llu, End time = %llu",
                   sensor_name, duration_us, start_time, end_time);

        // Validate echo duration is within expected range
        if (duration_us < MIN_ECHO_TIME)
        {
            log_remote("%s sensor: Echo too short (%.1f µs < %.1f µs) - Check for interference",
                       sensor_name, duration_us, (float)MIN_ECHO_TIME);
            continue; // Skip this sample
        }
        if (duration_us > MAX_ECHO_TIME)
        {
            log_remote("%s sensor: Echo too long (%.1f µs > %.1f µs) - Check for interference",
                       sensor_name, duration_us, (float)MAX_ECHO_TIME);
            continue; // Skip this sample
        }

        // 5. Calculate distance with improved precision
        float distance_cm = (duration_us * SPEED_OF_SOUND) / 2.0f;

        // 6. Validate final distance
        if (distance_cm < MIN_DISTANCE)
        {
            log_remote("%s sensor: Distance too small (%.1f cm < %.1f cm) - Object too close",
                       sensor_name, distance_cm, MIN_DISTANCE);
            continue; // Skip this sample
        }
        if (distance_cm > MAX_DISTANCE)
        {
            log_remote("%s sensor: Distance too large (%.1f cm > %.1f cm) - No object detected",
                       sensor_name, distance_cm, MAX_DISTANCE);
            continue; // Skip this sample
        }

        sum_distance += distance_cm;
        valid_samples++;
    }

    // Return average of valid samples, or -1 if no valid samples
    if (valid_samples > 0)
    {
        float avg_distance = sum_distance / valid_samples;
        log_remote("%s sensor: Final distance = %.1f cm (from %d valid samples)",
                   sensor_name, avg_distance, valid_samples);
        return avg_distance;
    }

    log_remote("%s sensor: No valid samples obtained - Check sensor wiring and power", sensor_name);
    return -1.0f;
}
