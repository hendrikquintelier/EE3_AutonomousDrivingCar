// motorcontrol.c

#include "motorcontrol.h"
#include "mpu.h"
#include "encoder.h"
#include "wifi_logger.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

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
#define HEADING_K 0.01f // Reduced from 0.05f for gentler heading correction

// -----------------------------------------------------
// PID Control Constants
// -----------------------------------------------------
#define HEADING_KP 0.05f           // Increased from 0.005f for stronger immediate response
#define HEADING_KI 0.0005f         // Increased from 0.0001f for better steady-state correction
#define HEADING_KD 0.004f          // Increased from 0.001f for better damping
#define HEADING_I_MAX 0.1f         // Increased from 0.05f to allow more integral correction
#define MIN_SPEED_FOR_HEADING 0.1f // Minimum speed before applying heading corrections

// -----------------------------------------------------
// PID State Variables
// -----------------------------------------------------
static float heading_integral = 0.0f;
static float last_heading_error = 0.0f;
static uint32_t last_heading_update = 0;

// -----------------------------------------------------
// Additional static variables
// -----------------------------------------------------
static bool is_braking_state = false;
static float current_speed_value = 0.0f;
static float last_left_speed = 0.0f;
static float last_right_speed = 0.0f;
static const float MAX_SPEED_CHANGE = 0.1f; // Maximum speed change per iteration
static bool first_run = true;

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
// Drive straight with enhanced heading hold
// -----------------------------------------------------
void motor_drive_straight(float speed)
{
    current_speed_value = speed;
    is_braking_state = false;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 1. Read the current orientation
    mpu_data_t mpu_data = mpu_get_orientation();
    float current_heading = mpu_data.yaw;

    // 2. Compute heading error relative to desired_heading
    float error = desired_heading - current_heading;

    // Normalize error to -180..+180 with improved handling of 360° wrap-around
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
    }

    // On first run, set the desired heading to match current heading
    if (first_run)
    {
        desired_heading = current_heading;
        error = 0.0f;
        first_run = false;
        my_wifi_log("Initial heading set to %.2f°\n", current_heading);
    }

    // 3. Calculate time delta for integral and derivative terms
    float dt = (current_time - last_heading_update) / 1000.0f; // Convert to seconds
    if (dt > 0.1f)
        dt = 0.1f; // Cap at 100ms
    last_heading_update = current_time;

    // 4. Update integral term with anti-windup and deadband
    if (speed > MIN_SPEED_FOR_HEADING)
    {
        heading_integral += error * dt;
        if (heading_integral > HEADING_I_MAX)
            heading_integral = HEADING_I_MAX;
        if (heading_integral < -HEADING_I_MAX)
            heading_integral = -HEADING_I_MAX;
    }

    // 5. Calculate derivative term with low-pass filtering
    float derivative = (error - last_heading_error) / dt;
    last_heading_error = error;

    // 6. Apply PID control with speed-based scaling
    float correction = 0.0f;
    if (speed > MIN_SPEED_FOR_HEADING)
    {
        // Scale correction based on error magnitude for more aggressive correction of larger errors
        float error_scale = fabsf(error) / 180.0f;    // Normalize to 0-1
        correction = (HEADING_KP * error +            // Proportional term
                      HEADING_KI * heading_integral + // Integral term
                      HEADING_KD * derivative) *      // Derivative term
                     (speed / 0.6f) *                 // Scale by current speed
                     (0.5f + 0.5f * error_scale);     // More aggressive for larger errors
    }

    // 7. Convert to left/right speeds with improved scaling
    float target_left_speed = speed + correction;
    float target_right_speed = speed - correction;

    // 8. Smoothly transition to target speeds
    float left_speed = last_left_speed;
    float right_speed = last_right_speed;

    // Smoothly adjust speeds with reduced maximum change
    if (target_left_speed > left_speed)
        left_speed = fminf(left_speed + MAX_SPEED_CHANGE * 0.5f, target_left_speed);
    else
        left_speed = fmaxf(left_speed - MAX_SPEED_CHANGE * 0.5f, target_left_speed);

    if (target_right_speed > right_speed)
        right_speed = fminf(right_speed + MAX_SPEED_CHANGE * 0.5f, target_right_speed);
    else
        right_speed = fmaxf(right_speed - MAX_SPEED_CHANGE * 0.5f, target_right_speed);

    // Store current speeds for next iteration
    last_left_speed = left_speed;
    last_right_speed = right_speed;

    // 9. Clamp speeds to [0..1] with improved handling
    if (left_speed < 0)
    {
        right_speed += left_speed; // Transfer lost power to other wheel
        left_speed = 0;
    }
    if (right_speed < 0)
    {
        left_speed += right_speed; // Transfer lost power to other wheel
        right_speed = 0;
    }
    if (left_speed > 1.0)
        left_speed = 1.0;
    if (right_speed > 1.0)
        right_speed = 1.0;

    // 10. Set direction pins for forward (both motors):
    gpio_set_level(INPUT1_PIN, 1); // Motor A forward
    gpio_set_level(INPUT2_PIN, 0);
    gpio_set_level(INPUT3_PIN, 1); // Motor B forward
    gpio_set_level(INPUT4_PIN, 0);

    // 11. Compute duty cycles from speeds
    uint32_t left_duty = (uint32_t)(left_speed * LEDC_MAX_DUTY);
    uint32_t right_duty = (uint32_t)(right_speed * LEDC_MAX_DUTY);

    // 12. Update LEDC duty for each motor
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, left_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, right_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

    // Debug print with enhanced information
    my_wifi_log("Heading=%.2f°, Error=%.2f, I=%.2f, D=%.2f => L=%.2f, R=%.2f\n",
                current_heading, error, heading_integral, derivative, left_speed, right_speed);
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
// Proportional brake: apply reverse torque based on wheel speed while maintaining heading
// -----------------------------------------------------
void motor_brake(void)
{
    is_braking_state = true;
    my_wifi_log("Proportional braking: applying reverse torque while maintaining heading...\n");

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
        // Get current heading and calculate heading correction
        mpu_data_t mpu_data = mpu_get_orientation();
        float current_heading = mpu_data.yaw;
        float error = desired_heading - current_heading;

        // Normalize error to -180..+180
        while (error > 180.0f)
            error -= 360.0f;
        while (error < -180.0f)
            error += 360.0f;

        // Calculate time delta for PID
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (current_time - last_heading_update) / 1000.0f;
        if (dt > 0.1f)
            dt = 0.1f;
        last_heading_update = current_time;

        // Update integral term with anti-windup
        heading_integral += error * dt;
        if (heading_integral > HEADING_I_MAX)
            heading_integral = HEADING_I_MAX;
        if (heading_integral < -HEADING_I_MAX)
            heading_integral = -HEADING_I_MAX;

        // Calculate derivative term
        float derivative = (error - last_heading_error) / dt;
        last_heading_error = error;

        // Apply PID control for heading correction
        float heading_correction = HEADING_KP * error +
                                   HEADING_KI * heading_integral +
                                   HEADING_KD * derivative;

        // Measure encoders for speed-based braking
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

        // Calculate base brake duty based on wheel speeds
        int speed1 = abs(delta1);
        int speed2 = abs(delta2);

        uint32_t base_duty1 = BRAKE_K * speed1;
        uint32_t base_duty2 = BRAKE_K * speed2;
        if (base_duty1 > LEDC_MAX_DUTY)
            base_duty1 = LEDC_MAX_DUTY;
        if (base_duty2 > LEDC_MAX_DUTY)
            base_duty2 = LEDC_MAX_DUTY;

        // Apply heading correction to brake duties
        float correction_factor = heading_correction * LEDC_MAX_DUTY;
        int32_t duty1 = base_duty1 + (int32_t)correction_factor;
        int32_t duty2 = base_duty2 - (int32_t)correction_factor;

        // Clamp duties to valid range
        if (duty1 < 0)
            duty1 = 0;
        if (duty2 < 0)
            duty2 = 0;
        if (duty1 > LEDC_MAX_DUTY)
            duty1 = LEDC_MAX_DUTY;
        if (duty2 > LEDC_MAX_DUTY)
            duty2 = LEDC_MAX_DUTY;

        // Update motor duties
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, duty1);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, duty2);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

        my_wifi_log("Brake => speed1=%d, speed2=%d, heading=%.2f°, error=%.2f => duty1=%ld, duty2=%ld\n",
                    speed1, speed2, current_heading, error, duty1, duty2);
    }

    // When braking is complete, reset the state
    is_braking_state = false;
}

float motor_get_current_speed(void)
{
    // Get encoder readings
    int enc1 = encoder_get_count(1);
    int enc2 = encoder_get_count(2);

    // Calculate average speed from both encoders
    float avg_speed = (abs(enc1) + abs(enc2)) / 2.0f;

    // Normalize to 0..1 range (assuming max encoder count of 1000)
    float normalized_speed = avg_speed / 1000.0f;
    if (normalized_speed > 1.0f)
        normalized_speed = 1.0f;

    current_speed_value = normalized_speed;
    return normalized_speed;
}

float motor_get_current_heading(void)
{
    // Get current heading from MPU
    mpu_data_t mpu_data = mpu_get_orientation();
    return mpu_data.yaw;
}

bool motor_is_braking(void)
{
    return is_braking_state;
}

// Modify the motor_stop function to update states
void motor_stop(void)
{
    current_speed_value = 0.0f;
    is_braking_state = false;

    // Set direction pins to stop
    gpio_set_level(INPUT1_PIN, 0);
    gpio_set_level(INPUT2_PIN, 0);
    gpio_set_level(INPUT3_PIN, 0);
    gpio_set_level(INPUT4_PIN, 0);

    // Set duty cycles to 0
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

    my_wifi_log("Motor stop complete.\n");
}
