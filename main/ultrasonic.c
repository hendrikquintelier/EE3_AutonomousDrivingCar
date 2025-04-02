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

    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
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

    // Take multiple samples and average them
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
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
            continue; // Skip this sample
        }
        end_time = esp_timer_get_time();

        // 4. Calculate duration and validate timing
        float duration_us = (float)(end_time - start_time);

        // Validate echo duration is within expected range
        if (duration_us < MIN_ECHO_TIME || duration_us > MAX_ECHO_TIME)
        {
            continue; // Skip this sample
        }

        // 5. Calculate distance with improved precision
        float distance_cm = (duration_us * SPEED_OF_SOUND) / 2.0f;

        // 6. Validate final distance
        if (distance_cm < MIN_DISTANCE || distance_cm > MAX_DISTANCE)
        {
            continue; // Skip this sample
        }

        sum_distance += distance_cm;
        valid_samples++;
    }

    // Return average of valid samples, or -1 if no valid samples
    if (valid_samples > 0)
    {
        return sum_distance / valid_samples;
    }
    return -1.0f;
}
