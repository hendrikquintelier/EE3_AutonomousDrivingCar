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
#define ENABLE_12_PIN 38 // Motor A enable (enable 1,2)
#define ENABLE_34_PIN 45 // Motor B enable (enable 3,4)

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
#define LEDC_FREQUENCY 1000 // Back to 1000 Hz
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
#define HEADING_KP 0.15f           // Increased from 0.08f for faster response
#define HEADING_KI 0.002f          // Increased from 0.001f for better steady-state
#define HEADING_KD 0.01f           // Increased from 0.006f for better damping
#define HEADING_I_MAX 0.2f         // Increased from 0.15f to allow more integral action
#define MIN_SPEED_FOR_HEADING 0.1f // Minimum speed before applying heading corrections

// -----------------------------------------------------
// Calibration Parameters
// -----------------------------------------------------
#define CALIBRATION_SPEED 0.4f   // Speed during calibration (40%)
#define CALIBRATION_TIME 5000    // Calibration duration in ms
#define CALIBRATION_SAMPLES 50   // Number of samples to collect
#define MIN_ERROR_THRESHOLD 0.5f // Minimum error to consider for calibration
#define MAX_OSCILLATION 2.0f     // Maximum allowed oscillation in degrees

// -----------------------------------------------------
// PID State Variables
// -----------------------------------------------------
static float heading_integral = 0.0f;
static float last_heading_error = 0.0f;
static uint32_t last_heading_update = 0;
static bool is_calibrating = false;
static float calibrated_kp = HEADING_KP;
static float calibrated_ki = HEADING_KI;
static float calibrated_kd = HEADING_KD;

// -----------------------------------------------------
// Additional static variables
// -----------------------------------------------------
static bool is_braking_state = false;
static float current_speed_value = 0.0f;

// -----------------------------------------------------
// Calibration Data Structure
// -----------------------------------------------------
typedef struct
{
    float error;
    float derivative;
    float integral;
    float correction;
} calibration_sample_t;

// -----------------------------------------------------
// Initialize the motor driver
// -----------------------------------------------------
void motor_init(void)
{
    my_wifi_log("Initializing motor driver...\n");

    // Configure direction pins as outputs
    gpio_reset_pin(INPUT1_PIN);
    gpio_reset_pin(INPUT2_PIN);
    gpio_reset_pin(INPUT3_PIN);
    gpio_reset_pin(INPUT4_PIN);

    gpio_set_direction(INPUT1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT4_PIN, GPIO_MODE_OUTPUT);

    // Initialize them LOW
    gpio_set_level(INPUT1_PIN, 0);
    gpio_set_level(INPUT2_PIN, 0);
    gpio_set_level(INPUT3_PIN, 0);
    gpio_set_level(INPUT4_PIN, 0);

    // Configure enable pins for PWM
    gpio_reset_pin(ENABLE_12_PIN);
    gpio_reset_pin(ENABLE_34_PIN);

    // LEDC timer config
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LEDC_FREQUENCY,
        .duty_resolution = LEDC_TIMER_BITS,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_config);

    // Configure LEDC channel for Motor A
    ledc_channel_config_t channel_conf_a = {
        .gpio_num = ENABLE_12_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_MOTOR_A,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channel_conf_a);

    // Configure LEDC channel for Motor B
    ledc_channel_config_t channel_conf_b = {
        .gpio_num = ENABLE_34_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_MOTOR_B,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channel_conf_b);

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

    // Get current heading
    mpu_data_t mpu_data = mpu_get_orientation();
    float current_heading = mpu_data.yaw;

    // Set direction pins for forward motion
    gpio_set_level(INPUT1_PIN, 1);
    gpio_set_level(INPUT2_PIN, 0);
    gpio_set_level(INPUT3_PIN, 1);
    gpio_set_level(INPUT4_PIN, 0);

    // Calculate duty cycle
    uint32_t duty = (uint32_t)(speed * LEDC_MAX_DUTY);

    // Set both motors to the same speed
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_A);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR_B);

    my_wifi_log("Motors set to speed %.2f (duty %lu), heading %.2f°\n",
                speed, duty, current_heading);
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

// -----------------------------------------------------
// Calibration Functions
// -----------------------------------------------------
static void analyze_calibration_data(calibration_sample_t *samples, int count)
{
    float max_error = 0.0f;
    float avg_error = 0.0f;
    float max_derivative = 0.0f;
    float oscillation = 0.0f;
    float steady_state_error = 0.0f;

    // Calculate statistics
    for (int i = 0; i < count; i++)
    {
        float abs_error = fabsf(samples[i].error);
        max_error = fmaxf(max_error, abs_error);
        avg_error += abs_error;
        max_derivative = fmaxf(max_derivative, fabsf(samples[i].derivative));

        // Calculate oscillation (peak-to-peak)
        if (i > 0)
        {
            float delta = fabsf(samples[i].error - samples[i - 1].error);
            oscillation = fmaxf(oscillation, delta);
        }
    }
    avg_error /= count;
    steady_state_error = samples[count - 1].error; // Final error represents steady-state

    // Adjust gains based on analysis
    if (max_error > 5.0f)
    {
        // System reacts too slowly or has large errors
        calibrated_kp *= 1.2f;
        my_wifi_log("Increasing KP (%.3f) due to large errors\n", calibrated_kp);
    }

    if (steady_state_error > MIN_ERROR_THRESHOLD)
    {
        // Steady-state error present
        calibrated_ki *= 1.5f;
        my_wifi_log("Increasing KI (%.3f) due to steady-state error\n", calibrated_ki);
    }

    if (oscillation > MAX_OSCILLATION)
    {
        // Too much oscillation
        calibrated_kd *= 1.3f;
        calibrated_kp *= 0.9f; // Reduce proportional gain
        my_wifi_log("Adjusting KD (%.3f) and KP (%.3f) due to oscillation\n",
                    calibrated_kd, calibrated_kp);
    }

    // Log calibration results
    my_wifi_log("Calibration Analysis:\n");
    my_wifi_log("Max Error: %.2f°\n", max_error);
    my_wifi_log("Avg Error: %.2f°\n", avg_error);
    my_wifi_log("Max Derivative: %.2f°/s\n", max_derivative);
    my_wifi_log("Oscillation: %.2f°\n", oscillation);
    my_wifi_log("Steady-state Error: %.2f°\n", steady_state_error);
    my_wifi_log("Final Gains - KP: %.3f, KI: %.3f, KD: %.3f\n",
                calibrated_kp, calibrated_ki, calibrated_kd);
}

void motor_calibrate_pid(void)
{
    if (is_calibrating)
    {
        my_wifi_log("Calibration already in progress!\n");
        return;
    }

    my_wifi_log("Starting PID calibration...\n");
    is_calibrating = true;

    // Reset calibration values to defaults
    calibrated_kp = HEADING_KP;
    calibrated_ki = HEADING_KI;
    calibrated_kd = HEADING_KD;
    heading_integral = 0.0f;
    last_heading_error = 0.0f;

    // Allocate memory for calibration samples
    calibration_sample_t *samples = malloc(sizeof(calibration_sample_t) * CALIBRATION_SAMPLES);
    if (!samples)
    {
        my_wifi_log("Failed to allocate calibration samples!\n");
        is_calibrating = false;
        return;
    }

    // Set initial heading
    mpu_data_t mpu_data = mpu_get_orientation();
    motor_set_desired_heading(mpu_data.yaw);

    // Start driving at calibration speed
    motor_drive_straight(CALIBRATION_SPEED);

    // Collect samples
    int sample_count = 0;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (sample_count < CALIBRATION_SAMPLES)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - start_time > CALIBRATION_TIME)
        {
            my_wifi_log("Calibration timeout!\n");
            break;
        }

        // Get current heading and calculate error
        mpu_data = mpu_get_orientation();
        float current_heading = mpu_data.yaw;
        float error = motor_get_current_heading() - current_heading;

        // Normalize error to [-180, +180]
        if (error > 180.0f)
            error -= 360.0f;
        if (error < -180.0f)
            error += 360.0f;

        // Calculate derivative
        float dt = 0.01f; // Fixed dt for calibration
        float derivative = (error - last_heading_error) / dt;
        last_heading_error = error;

        // Update integral
        heading_integral += error * dt;

        // Store sample
        samples[sample_count].error = error;
        samples[sample_count].derivative = derivative;
        samples[sample_count].integral = heading_integral;
        samples[sample_count].correction = (calibrated_kp * error +
                                            calibrated_ki * heading_integral +
                                            calibrated_kd * derivative);

        sample_count++;
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms between samples
    }

    // Stop the car
    motor_stop();

    // Analyze the collected data
    analyze_calibration_data(samples, sample_count);

    // Free allocated memory
    free(samples);
    is_calibrating = false;

    my_wifi_log("PID calibration complete!\n");
}
