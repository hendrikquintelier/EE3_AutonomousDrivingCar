// motorcontrol.c

#include "motorcontrol.h"
#include "mpu.h"
#include "encoder.h"
#include "wifi_logger.h"
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// -----------------------------------------------------
// L293D Direction Pins
// -----------------------------------------------------
#define INPUT1_PIN 37 // Motor A input 1
#define INPUT2_PIN 48 // Motor A input 2
#define INPUT3_PIN 35 // Motor B input 3
#define INPUT4_PIN 36 // Motor B input 4

// -----------------------------------------------------
// L293D Enable Pins (GPIO numbers)
// -----------------------------------------------------
#define ENABLE_12_PIN 38 // Motor A enable
#define ENABLE_34_PIN 45 // Motor B enable

// -----------------------------------------------------
// LEDC Channels (software channels 0..7)
// -----------------------------------------------------
#define LEDC_CHANNEL_MOTOR_A LEDC_CHANNEL_0
#define LEDC_CHANNEL_MOTOR_B LEDC_CHANNEL_1

// -----------------------------------------------------
// LEDC Config
// -----------------------------------------------------
#define LEDC_MODE LEDC_LOW_SPEED_MODE // ESP32-S3 typically only has low-speed mode
#define LEDC_TIMER_BITS 13
#define LEDC_FREQUENCY 1000 // 1 kHz
#define LEDC_MAX_DUTY ((1 << LEDC_TIMER_BITS) - 1)

// -----------------------------------------------------
// Global Heading State
// -----------------------------------------------------
static float desired_heading = 0.0f;

// -----------------------------------------------------
// Tuning Constants
// -----------------------------------------------------
#define BRAKE_K 50      // Proportional gain for braking
#define HEADING_K 0.05f // Simple heading correction gain

// -----------------------------------------------------
// Initialize the motor driver
// -----------------------------------------------------
void motor_init(void)
{
    my_wifi_log("Initializing motor driver...\n");

    // 1. Reset & configure direction pins as outputs
    gpio_reset_pin(INPUT1_PIN);
    gpio_reset_pin(INPUT2_PIN);
    gpio_reset_pin(INPUT3_PIN);
    gpio_reset_pin(INPUT4_PIN);

    esp_err_t ret;

    ret = gpio_set_direction(INPUT1_PIN, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
        my_wifi_log("Error setting INPUT1_PIN direction\n");

    ret = gpio_set_direction(INPUT2_PIN, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
        my_wifi_log("Error setting INPUT2_PIN direction\n");

    ret = gpio_set_direction(INPUT3_PIN, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
        my_wifi_log("Error setting INPUT3_PIN direction\n");

    ret = gpio_set_direction(INPUT4_PIN, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
        my_wifi_log("Error setting INPUT4_PIN direction\n");

    // Initialize them LOW
    gpio_set_level(INPUT1_PIN, 0);
    gpio_set_level(INPUT2_PIN, 0);
    gpio_set_level(INPUT3_PIN, 0);
    gpio_set_level(INPUT4_PIN, 0);

    // 2. Configure the enable pins (GPIO) for LEDC PWM
    gpio_reset_pin(ENABLE_12_PIN);
    gpio_reset_pin(ENABLE_34_PIN);

    // LEDC timer config
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};

    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK)
        my_wifi_log("Error configuring LEDC timer\n");

    // 3. Configure LEDC channel for Motor A
    ledc_channel_config_t channel_conf_a = {
        .gpio_num = ENABLE_12_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_MOTOR_A,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0};

    ret = ledc_channel_config(&channel_conf_a);
    if (ret != ESP_OK)
        my_wifi_log("Error configuring LEDC channel A\n");

    // 4. Configure LEDC channel for Motor B
    ledc_channel_config_t channel_conf_b = {
        .gpio_num = ENABLE_34_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_MOTOR_B,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0};

    ret = ledc_channel_config(&channel_conf_b);
    if (ret != ESP_OK)
        my_wifi_log("Error configuring LEDC channel B\n");

    my_wifi_log("Motor driver initialization complete.\n");
}

// -----------------------------------------------------
// Set the desired heading
// -----------------------------------------------------
void motor_set_desired_heading(float heading)
{
    desired_heading = heading;
    my_wifi_log("Desired heading set to %.2f°\n", heading);
}

// -----------------------------------------------------
// Drive straight with heading hold
// -----------------------------------------------------
void motor_drive_straight(float speed)
{
    static uint32_t last_boost_time = 0;
    static bool boost_done = false;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // If this is the first call or we haven't done the boost yet
    if (!boost_done)
    {
        my_wifi_log("Applying startup boost...\n");

        // Set direction pins for forward (both motors):
        gpio_set_level(INPUT1_PIN, 1); // Motor A forward
        gpio_set_level(INPUT2_PIN, 0);
        gpio_set_level(INPUT3_PIN, 1); // Motor B forward
        gpio_set_level(INPUT4_PIN, 0);

        // Set full speed for boost
        uint32_t boost_duty = LEDC_MAX_DUTY;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, boost_duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, boost_duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

        // Wait for boost duration
        vTaskDelay(pdMS_TO_TICKS(250));

        // Mark boost as done
        boost_done = true;
        last_boost_time = current_time;
    }

    // After boost, maintain constant speed
    my_wifi_log("Setting motor speed to %.2f\n", speed);

    // 1. Read the current orientation
    mpu_data_t mpu_data = mpu_get_orientation();
    float current_heading = mpu_data.yaw;

    // 2. Compute heading error relative to desired_heading
    float error = desired_heading - current_heading;
    // Normalize error to -180..+180
    while (error > 180.0f)
        error -= 360.0f;
    while (error < -180.0f)
        error += 360.0f;

    // 3. Apply a simple P-controller to get a correction
    float correction = HEADING_K * error;

    // 4. Convert to left/right speeds
    float left_speed = speed + correction;
    float right_speed = speed - correction;

    // 5. Clamp speeds to [0..1]
    if (left_speed < 0)
        left_speed = 0;
    if (left_speed > 1.0)
        left_speed = 1.0;
    if (right_speed < 0)
        right_speed = 0;
    if (right_speed > 1.0)
        right_speed = 1.0;

    // 6. Set direction pins for forward (both motors):
    gpio_set_level(INPUT1_PIN, 1); // Motor A forward
    gpio_set_level(INPUT2_PIN, 0);
    gpio_set_level(INPUT3_PIN, 1); // Motor B forward
    gpio_set_level(INPUT4_PIN, 0);

    // 7. Compute duty cycles from speeds
    uint32_t left_duty = (uint32_t)(left_speed * LEDC_MAX_DUTY);
    uint32_t right_duty = (uint32_t)(right_speed * LEDC_MAX_DUTY);

    my_wifi_log("Setting motor duties - Left: %lu, Right: %lu\n", left_duty, right_duty);

    // 8. Update LEDC duty for each motor
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, left_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, right_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

    // Debug print
    my_wifi_log("Heading=%.2f°, Error=%.2f => L=%.2f, R=%.2f\n",
                current_heading, error, left_speed, right_speed);
}

// -----------------------------------------------------
// Turn by angle (blocking) with a simple spin approach
// -----------------------------------------------------
void motor_turn_by_angle(float angle, float speed)
{
    mpu_data_t data = mpu_get_orientation();
    float start_heading = data.yaw;
    float target_heading = start_heading + angle;

    // Normalize target heading
    while (target_heading >= 360.0f)
        target_heading -= 360.0f;
    while (target_heading < 0.0f)
        target_heading += 360.0f;

    float tolerance = 3.0f; // degrees

    // Example approach: left motor forward, right motor backward at 50% duty
    float left_speed = 0.5f;
    float right_speed = -0.5f;

    // In real code, set direction pins and LEDC duty accordingly for each motor

    while (1)
    {
        mpu_data_t d = mpu_get_orientation();
        float curr = d.yaw;

        float diff = target_heading - curr;
        // normalize
        while (diff > 180.0f)
            diff -= 360.0f;
        while (diff < -180.0f)
            diff += 360.0f;

        if (fabsf(diff) < tolerance)
        {
            // Stop turning
            motor_brake(); // calls active braking
            my_wifi_log("Turn complete. Now at heading=%.2f°\n", curr);
            break;
        }

        // Keep turning
        // e.g. set the direction pins for spin in place
        // set LEDC duty = 50% on each side (one reversed)
        // ...
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// -----------------------------------------------------
// Proportional brake: apply reverse torque based on wheel speed
// -----------------------------------------------------
void motor_brake(void)
{
    my_wifi_log("Proportional braking: applying reverse torque based on wheel speed...\n");

    // 1. Set direction pins to reverse
    // Motor A: IN1=LOW, IN2=HIGH
    gpio_set_level(INPUT1_PIN, 0);
    gpio_set_level(INPUT2_PIN, 1);

    // Motor B: IN3=LOW, IN4=HIGH
    gpio_set_level(INPUT3_PIN, 0);
    gpio_set_level(INPUT4_PIN, 1);

    // Start with duty=0 on both channels
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

    while (1)
    {
        // Measure encoders
        int enc1_before = encoder_get_count(1);
        int enc2_before = encoder_get_count(2);

        vTaskDelay(pdMS_TO_TICKS(50));

        int enc1_after = encoder_get_count(1);
        int enc2_after = encoder_get_count(2);

        int delta1 = enc1_after - enc1_before;
        int delta2 = enc2_after - enc2_before;

        // If both wheels are not moving => stop
        if (delta1 == 0 && delta2 == 0)
        {
            my_wifi_log("Car is fully stopped.\n");

            // Turn everything off
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

            gpio_set_level(INPUT1_PIN, 0);
            gpio_set_level(INPUT2_PIN, 0);
            gpio_set_level(INPUT3_PIN, 0);
            gpio_set_level(INPUT4_PIN, 0);

            break;
        }

        // Calculate speed => duty
        int speed1 = abs(delta1);
        int speed2 = abs(delta2);

        uint32_t duty1 = BRAKE_K * speed1;
        uint32_t duty2 = BRAKE_K * speed2;
        if (duty1 > LEDC_MAX_DUTY)
            duty1 = LEDC_MAX_DUTY;
        if (duty2 > LEDC_MAX_DUTY)
            duty2 = LEDC_MAX_DUTY;

        // Update duty
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, duty1);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, duty2);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

        my_wifi_log("Brake => speed1=%d, speed2=%d => duty1=%lu, duty2=%lu\n",
                    speed1, speed2, (unsigned long)duty1, (unsigned long)duty2);
    }
}
