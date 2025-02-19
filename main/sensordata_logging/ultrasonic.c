#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include <portmacro.h>

#define TRIG_LEFT 7
#define ECHO_LEFT 4
#define TRIG_FRONT 15
#define ECHO_FRONT 5
#define TRIG_RIGHT 16
#define ECHO_RIGHT 6

#define CONFIG_FREERTOS_HZ 100

void init_ultrasonic_sensor(gpio_num_t trig, gpio_num_t echo)
{
    gpio_reset_pin(trig);
    gpio_set_direction(trig, GPIO_MODE_OUTPUT);
    gpio_set_level(trig, 0);

    gpio_reset_pin(echo);
    gpio_set_direction(echo, GPIO_MODE_INPUT);
}

float get_distance(gpio_num_t trig, gpio_num_t echo)
{
    uint64_t start_time = 0, end_time = 0;

    // Send a 10us pulse to trigger pin
    gpio_set_level(trig, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(trig, 0);

    // Wait for echo pin to go high
    while (gpio_get_level(echo) == 0)
        ;

    start_time = esp_timer_get_time(); // Start timing

    // Wait for echo pin to go low
    while (gpio_get_level(echo) == 1)
        ;

    end_time = esp_timer_get_time(); // End timing

    // Calculate distance (speed of sound = 34300 cm/s)
    float distance = ((end_time - start_time) * 0.0343) / 2;

    return distance;
}

void ultrasonic_task(void *pvParameters)
{
    while (1)
    {
        float distance_left = get_distance(TRIG_LEFT, ECHO_LEFT);
        float distance_front = get_distance(TRIG_FRONT, ECHO_FRONT);
        float distance_right = get_distance(TRIG_RIGHT, ECHO_RIGHT);

        printf("Left: %.2f cm | Front: %.2f cm | Right: %.2f cm\n",
               distance_left, distance_front, distance_right);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    init_ultrasonic_sensor(TRIG_LEFT, ECHO_LEFT);
    init_ultrasonic_sensor(TRIG_FRONT, ECHO_FRONT);
    init_ultrasonic_sensor(TRIG_RIGHT, ECHO_RIGHT);

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);
}
