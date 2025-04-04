// motorcontrol.c

#include "motorcontrol.h"
#include "wifi_logger.h"
#include "mpu.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "esp_mac.h"

// ============================================================================
// Hardware Configuration
// ============================================================================
// Motor A pins (Left motor)
#define MOTOR_A_IN1 37 // Direction pin 1
#define MOTOR_A_IN2 48 // Direction pin 2
#define MOTOR_A_EN 38  // Enable pin (PWM)

// Motor B pins (Right motor)
#define MOTOR_B_IN1 35 // Direction pin 1
#define MOTOR_B_IN2 36 // Direction pin 2
#define MOTOR_B_EN 45  // Enable pin (PWM)

// PWM Configuration
#define PWM_CHANNEL_A LEDC_CHANNEL_0
#define PWM_CHANNEL_B LEDC_CHANNEL_1
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION 13
#define PWM_FREQ 5000
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION) - 1) // Maximum PWM duty cycle value (8191 for 13-bit)

// ============================================================================
// PID Control Parameters
// ============================================================================
#define PID_KP 0.03f         // Further reduced for gentler response
#define PID_KI 0.0002f       // Reduced integral action
#define PID_KD 0.00035f      // Increased derivative gain for better damping
#define PID_I_MAX 0.03f      // Reduced integral windup limit
#define PID_UPDATE_MS 10     // Reduced update interval for more responsive control
#define DRIVE_TIME_MS 5000   // Total drive time (10 seconds)
#define PID_DEADZONE 0.25f   // Reduced deadzone to 0.5 degrees for more precise control
#define MAX_CORRECTION 0.3f  // Reduced maximum correction to 15% of base speed
#define MIN_DUTY_RATIO 0.05f // Increased minimum duty ratio to 40%
#define MAX_DUTY_RATIO 0.9f  // Set maximum duty ratio to 90% of PWM_MAX_DUTY

// Add these defines at the top with other defines
#define TURN_PID_KP 0.3f            // Higher proportional gain for faster response
#define TURN_PID_KI 0.01f           // Small integral to handle steady-state error
#define TURN_PID_KD 0.05f           // Derivative to dampen oscillations
#define TURN_PID_I_MAX 0.5f         // Maximum integral term
#define TURN_SPEED 0.65f            // Increased from 0.5f to 0.65f (80% power)
#define TURN_TOLERANCE 2.0f         // Reduced from 5.0f to 2.0f for more precise turns
#define TURN_TIMEOUT_MS 3000        // Maximum time to complete turn
#define MIN_TURN_TIME_MS 300        // Minimum time to ensure full rotation
#define CALIBRATION_SPEED 0.3f      // Speed for motor calibration
#define CALIBRATION_TIME_MS 2000    // Time to run calibration
#define MIN_MOTOR_FORCE 0.4f        // Increased from 0.2f to 0.4f (40% power)
#define TURN_DEADZONE 1.0f          // Degrees of deadzone to prevent oscillations
#define TURN_MAX_CORRECTION 0.3f    // Maximum correction factor (30% of base speed)
#define TURN_SCALE_FACTOR 0.5f      // Scale factor for corrections near target
#define MOMENTUM_THRESHOLD 45.0f    // Degrees at which we switch to momentum mode
#define MOMENTUM_SPEED 0.3f         // Increased from 0.2f to 0.3f (60% power)
#define ANGULAR_VEL_THRESHOLD 50.0f // Degrees/second threshold for overshoot prediction
#define BRAKE_FORCE 0.6f            // Increased from 0.4f to 0.6f (60% power)
#define MAX_DERIVATIVE 100.0f       // Maximum derivative term for rate limiting

// ============================================================================
// State Variables
// ============================================================================
static bool is_braking = false;
static float current_speed = 0.0f;
static float target_yaw = 0.0f;
static float yaw_integral = 0.0f;
static float last_yaw_error = 0.0f;
static unsigned long last_pid_update = 0;

// Add these global variables at the top with other state variables
turn_control_t turn_control = {0};
motor_calibration_t motor_calibration = {
    .left_factor = 1.0f,
    .right_factor = 1.0f,
    .is_calibrated = false};

// Add these at the top with other state variables
static struct
{
    float yaw;
    float error;
    float correction;
    float left_dist;
    float right_dist;
    float distance;
    unsigned long duty_a;
    unsigned long duty_b;
    bool new_data;
} log_data = {0};

// Add this structure near the top with other state variables
static struct
{
    float left;
    float right;
    bool new_data;
    SemaphoreHandle_t mutex;
} ultrasonic_cache = {0};

// ============================================================================
// Helper Functions
// ============================================================================
static void set_motor_direction(bool forward)
{
    if (forward)
    {
        // Both motors forward
        gpio_set_level(MOTOR_A_IN1, 1);
        gpio_set_level(MOTOR_A_IN2, 0);
        gpio_set_level(MOTOR_B_IN1, 1);
        gpio_set_level(MOTOR_B_IN2, 0);
    }
    else
    {
        // Both motors backward
        gpio_set_level(MOTOR_A_IN1, 0);
        gpio_set_level(MOTOR_A_IN2, 1);
        gpio_set_level(MOTOR_B_IN1, 0);
        gpio_set_level(MOTOR_B_IN2, 1);
    }
}

static void set_motor_turn(bool turn_right)
{
    if (turn_right)
    {
        // Right motor forward, left motor backward
        gpio_set_level(MOTOR_A_IN1, 0); // Left motor backward
        gpio_set_level(MOTOR_A_IN2, 1);
        gpio_set_level(MOTOR_B_IN1, 1); // Right motor forward
        gpio_set_level(MOTOR_B_IN2, 0);
    }
    else
    {
        // Right motor backward, left motor forward
        gpio_set_level(MOTOR_A_IN1, 1); // Left motor forward
        gpio_set_level(MOTOR_A_IN2, 0);
        gpio_set_level(MOTOR_B_IN1, 0); // Right motor backward
        gpio_set_level(MOTOR_B_IN2, 1);
    }
}

static void set_motor_speed(long duty_a, long duty_b)
{
    // Handle negative duties by inverting direction
    bool motor_a_forward = true;
    bool motor_b_forward = true;

    // Convert signed duties to absolute values and set direction
    if (duty_a < 0)
    {
        motor_a_forward = false;
        duty_a = -duty_a;
    }
    if (duty_b < 0)
    {
        motor_b_forward = false;
        duty_b = -duty_b;
    }

    // Set motor directions
    gpio_set_level(MOTOR_A_IN1, motor_a_forward ? 1 : 0);
    gpio_set_level(MOTOR_A_IN2, motor_a_forward ? 0 : 1);
    gpio_set_level(MOTOR_B_IN1, motor_b_forward ? 1 : 0);
    gpio_set_level(MOTOR_B_IN2, motor_b_forward ? 0 : 1);

    // Clamp duties to valid range
    duty_a = (duty_a > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty_a;
    duty_b = (duty_b > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty_b;

    // Update PWM duties
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, duty_a);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, duty_b);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
}

static void stop_motors(void)
{
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_motor_speed(0, 0);
}

static float calculate_heading_correction(float error, float dt)
{
    // Prevent division by zero or very small dt
    if (dt < 0.001f) // Less than 1ms
    {
        dt = 0.001f;
    }

    // Apply deadzone to prevent small oscillations
    if (fabs(error) < PID_DEADZONE)
    {
        error = 0;
        yaw_integral = 0; // Reset integral in deadzone
    }

    // Update integral with anti-windup
    yaw_integral += error * dt;
    if (yaw_integral > PID_I_MAX)
        yaw_integral = PID_I_MAX;
    else if (yaw_integral < -PID_I_MAX)
        yaw_integral = -PID_I_MAX;

    // Calculate derivative term with rate limiting
    float derivative = (error - last_yaw_error) / dt;
    if (derivative > MAX_DERIVATIVE)
        derivative = MAX_DERIVATIVE;
    else if (derivative < -MAX_DERIVATIVE)
        derivative = -MAX_DERIVATIVE;

    // Calculate PID output
    float output = PID_KP * error + PID_KI * yaw_integral + PID_KD * derivative;

    // Store error for next iteration
    last_yaw_error = error;

    // Return the correction
    return output;
}

static float calculate_turn_correction(float error, float dt)
{
    // Update integral with anti-windup
    turn_control.integral += error * dt;
    turn_control.integral = (turn_control.integral > TURN_PID_I_MAX) ? TURN_PID_I_MAX : (turn_control.integral < -TURN_PID_I_MAX) ? -TURN_PID_I_MAX
                                                                                                                                  : turn_control.integral;

    // Calculate derivative
    float derivative = (error - turn_control.last_error) / dt;
    turn_control.last_error = error;

    // Apply deadzone to prevent small oscillations
    if (fabs(error) < TURN_DEADZONE)
    {
        error = 0;
        turn_control.integral = 0; // Reset integral in deadzone
    }

    // Calculate base correction
    float correction = TURN_PID_KP * error + TURN_PID_KI * turn_control.integral + TURN_PID_KD * derivative;

    // Scale down correction when close to target
    float error_scale = 1.0f;
    if (fabs(error) < 10.0f)
    { // Within 10 degrees of target
        error_scale = TURN_SCALE_FACTOR;
    }

    // Apply scaling
    correction *= error_scale;

    // Limit maximum correction
    if (correction > TURN_MAX_CORRECTION)
    {
        correction = TURN_MAX_CORRECTION;
    }
    else if (correction < -TURN_MAX_CORRECTION)
    {
        correction = -TURN_MAX_CORRECTION;
    }

    return correction;
}

// Add this helper function implementation
static void normalize_angle(float *angle)
{
    while (*angle > 180.0f)
        *angle -= 360.0f;
    while (*angle < -180.0f)
        *angle += 360.0f;
}

static void calibrate_motors(void)
{
    log_remote("Starting motor calibration...");

    // Set both motors to same speed
    unsigned long base_duty = (unsigned long)(CALIBRATION_SPEED * PWM_MAX_DUTY);
    set_motor_direction(true);
    set_motor_speed(base_duty, base_duty);

    // Get initial yaw
    float start_yaw = mpu_get_orientation().yaw;
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Run motors for calibration time
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < CALIBRATION_TIME_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Get final yaw
    float end_yaw = mpu_get_orientation().yaw;
    float yaw_diff = end_yaw - start_yaw;
    normalize_angle(&yaw_diff);

    // If we turned right, left motor is stronger
    // If we turned left, right motor is stronger
    if (yaw_diff > 0)
    {
        motor_calibration.left_factor = 1.0f;
        motor_calibration.right_factor = 1.0f - (yaw_diff / 90.0f);
    }
    else
    {
        motor_calibration.left_factor = 1.0f + (yaw_diff / 90.0f);
        motor_calibration.right_factor = 1.0f;
    }

    // Ensure factors are reasonable
    motor_calibration.left_factor = (motor_calibration.left_factor < 0.5f) ? 0.5f : (motor_calibration.left_factor > 1.5f) ? 1.5f
                                                                                                                           : motor_calibration.left_factor;
    motor_calibration.right_factor = (motor_calibration.right_factor < 0.5f) ? 0.5f : (motor_calibration.right_factor > 1.5f) ? 1.5f
                                                                                                                              : motor_calibration.right_factor;

    motor_calibration.is_calibrated = true;
    log_remote("Motor calibration complete. Left factor: %.2f, Right factor: %.2f",
               motor_calibration.left_factor, motor_calibration.right_factor);

    // Stop motors
    motor_stop();
}

// Helper: Delay in milliseconds
static void delay_ms(unsigned long ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// Add this function before log_task
static void ultrasonic_task(void *pvParameters)
{
    while (1)
    {
        // Get ultrasonic readings
        ultrasonic_readings_t ultrasonic = ultrasonic_get_all();

        // Update cache with mutex protection
        if (xSemaphoreTake(ultrasonic_cache.mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            ultrasonic_cache.left = ultrasonic.left;
            ultrasonic_cache.right = ultrasonic.right;
            ultrasonic_cache.new_data = true;
            xSemaphoreGive(ultrasonic_cache.mutex);
        }

        // Wait 500ms before next reading
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Modify log_task to use cached ultrasonic values
static void log_task(void *pvParameters)
{
    unsigned long last_log_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long log_count = 0;
    unsigned long last_rate_time = last_log_time;
    unsigned long last_status_time = last_log_time;

    while (1)
    {
        if (log_data.new_data)
        {
            unsigned long current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            unsigned long time_since_last = current_time - last_log_time;

            // Only log if enough time has passed
            if (time_since_last >= 100) // 10Hz target
            {
                // Get cached ultrasonic readings
                if (xSemaphoreTake(ultrasonic_cache.mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    log_data.left_dist = ultrasonic_cache.left;
                    log_data.right_dist = ultrasonic_cache.right;
                    xSemaphoreGive(ultrasonic_cache.mutex);
                }

                log_data.distance = mpu_get_filtered_frontal_distance();

                // Log status data
                log_remote("Yaw: %.2f°, Error: %.2f°, Correction: %.2f, Left: %.1fcm, Right: %.1fcm, Distance: %.1fcm, Duty A: %lu, Duty B: %lu",
                           log_data.yaw, log_data.error, log_data.correction,
                           log_data.left_dist, log_data.right_dist,
                           log_data.distance,
                           log_data.duty_a, log_data.duty_b);

                last_log_time = current_time;
                log_count++;

                // Calculate and log rate every second
                if (current_time - last_rate_time >= 1000)
                {
                    float rate = (float)log_count * 1000.0f / (current_time - last_rate_time);
                    log_remote("Logging rate: %.1f Hz, Time since last: %lu ms", rate, time_since_last);
                    log_count = 0;
                    last_rate_time = current_time;
                }
            }
            log_data.new_data = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Minimal delay to prevent watchdog issues
    }
}

// ============================================================================
// Public Interface Functions
// ============================================================================
void motor_init(void)
{
    log_remote("Initializing motor driver...");

    // Configure direction pins
    gpio_config_t dir_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&dir_conf);

    // Configure PWM timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    // Configure PWM channels
    ledc_channel_config_t channel_conf = {
        .gpio_num = MOTOR_A_EN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_A,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channel_conf);
    channel_conf.gpio_num = MOTOR_B_EN;
    channel_conf.channel = PWM_CHANNEL_B;
    ledc_channel_config(&channel_conf);

    // Initialize state
    current_speed = 0.0f;
    is_braking = false;
    yaw_integral = 0.0f;
    last_yaw_error = 0.0f;
    last_pid_update = 0;
    stop_motors();

    log_remote("Motor driver initialization complete");
}

void motor_forward_constant_speed(float speed)
{
    // Input validation
    speed = (speed < 0.0f) ? 0.0f : (speed > 1.0f) ? 1.0f
                                                   : speed;

    // Create mutex for ultrasonic cache
    ultrasonic_cache.mutex = xSemaphoreCreateMutex();

    // Create tasks
    TaskHandle_t log_task_handle;
    TaskHandle_t ultrasonic_task_handle;
    xTaskCreate(log_task, "log_task", 4096, NULL, 1, &log_task_handle);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 1, &ultrasonic_task_handle);

    // Initialize drive parameters
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long current_time = start_time;
    target_yaw = mpu_get_orientation().yaw;
    mpu_reset_movement();
    log_remote("Starting straight drive at %.0f%% speed, target yaw: %.2f°", speed * 100, target_yaw);

    // Set up motors for forward motion
    set_motor_direction(true);
    unsigned long base_duty = (unsigned long)(speed * PWM_MAX_DUTY);
    set_motor_speed(base_duty, base_duty);

    // Main drive loop
    unsigned long last_loop_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long loop_count = 0;
    unsigned long last_diag_time = last_loop_time;
    unsigned long last_mpu_time = last_loop_time;

    while (current_time - start_time < DRIVE_TIME_MS)
    {
        // Get current orientation
        mpu_data_t orientation = mpu_get_orientation();
        float current_yaw = orientation.yaw;

        // Calculate heading error (-180 to 180 degrees)
        float yaw_error = target_yaw - current_yaw;
        while (yaw_error > 180.0f)
            yaw_error -= 360.0f;
        while (yaw_error < -180.0f)
            yaw_error += 360.0f;

        // Update PID control
        unsigned long new_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (new_time - last_pid_update) / 1000.0f;
        last_pid_update = new_time;

        // Calculate heading correction
        float correction = calculate_heading_correction(yaw_error, dt);

        // Apply correction to motor speeds with constraints
        float duty_ratio_a = speed + correction;
        float duty_ratio_b = speed - correction;

        // Clamp duty ratios
        duty_ratio_a = (duty_ratio_a < MIN_DUTY_RATIO) ? MIN_DUTY_RATIO : (duty_ratio_a > MAX_DUTY_RATIO) ? MAX_DUTY_RATIO
                                                                                                          : duty_ratio_a;
        duty_ratio_b = (duty_ratio_b < MIN_DUTY_RATIO) ? MIN_DUTY_RATIO : (duty_ratio_b > MAX_DUTY_RATIO) ? MAX_DUTY_RATIO
                                                                                                          : duty_ratio_b;

        // Convert to duty cycles (ensure proper rounding and range)
        unsigned long duty_a = (unsigned long)(duty_ratio_a * PWM_MAX_DUTY + 0.5f);
        unsigned long duty_b = (unsigned long)(duty_ratio_b * PWM_MAX_DUTY + 0.5f);

        // Ensure duty cycles don't exceed maximum
        duty_a = (duty_a > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty_a;
        duty_b = (duty_b > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty_b;

        // Set motor speeds with different duty cycles for correction
        set_motor_speed(duty_a, duty_b);

        // Update log data without waiting
        log_data.yaw = current_yaw;
        log_data.error = yaw_error;
        log_data.correction = correction;
        log_data.duty_a = duty_a;
        log_data.duty_b = duty_b;
        log_data.new_data = true;

        // Update time and minimal delay
        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Calculate and log loop timing every second
        loop_count++;
        if (current_time - last_diag_time >= 1000)
        {
            float loop_rate = (float)loop_count * 1000.0f / (current_time - last_diag_time);
            float loop_time = (float)(current_time - last_loop_time);
            float mpu_time = (float)(current_time - last_mpu_time);
            log_remote("Control loop: %.1f Hz, Loop time: %.1f ms, MPU time: %.1f ms",
                       loop_rate, loop_time, mpu_time);
            loop_count = 0;
            last_diag_time = current_time;
        }
        last_loop_time = current_time;
        last_mpu_time = current_time;

        vTaskDelay(pdMS_TO_TICKS(1)); // Minimal delay to prevent watchdog issues
    }

    // Drive complete
    log_remote("Drive time elapsed, stopping motors...");
    motor_stop();

    // Delete both tasks
    vTaskDelete(log_task_handle);
    vTaskDelete(ultrasonic_task_handle);
    vSemaphoreDelete(ultrasonic_cache.mutex);
}

void motor_stop(void)
{
    current_speed = 0.0f;
    is_braking = false;
    yaw_integral = 0.0f;
    last_yaw_error = 0.0f;
    stop_motors();
    log_remote("Motor stop complete");
}

void motor_turn_90(bool turn_right)
{
    // Initialize turn parameters
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float initial_yaw = mpu_get_orientation().yaw;
    float target_yaw = initial_yaw + (turn_right ? -90.0f : 90.0f);
    normalize_angle(&target_yaw);

    // Variables for momentum control
    float last_yaw = initial_yaw;
    unsigned long last_yaw_time = start_time;
    float angular_velocity = 0.0f;
    bool momentum_mode = false;
    bool braking = false;

    // Set up motors for turning with full power initially
    set_motor_turn(turn_right);
    unsigned long base_duty = (unsigned long)(TURN_SPEED * PWM_MAX_DUTY);

    // Log initial motor setup
    log_remote("Starting turn with base_duty: %lu (%.1f%%)", base_duty, TURN_SPEED * 100.0f);

    // Set initial motor speeds
    set_motor_speed(base_duty, base_duty);

    // Initialize PID control
    turn_control.integral = 0.0f;
    turn_control.last_error = 0.0f;
    turn_control.target_yaw = target_yaw;
    turn_control.last_update = start_time;
    turn_control.is_turning = true;

    // Main turn loop
    while (turn_control.is_turning)
    {
        // Get current orientation and time
        mpu_data_t orientation = mpu_get_orientation();
        float current_yaw = orientation.yaw;
        unsigned long current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Calculate angular velocity
        float dt = (current_time - last_yaw_time) / 1000.0f;
        if (dt > 0.01f)
        { // Only calculate if we have enough time difference
            float yaw_diff = current_yaw - last_yaw;
            normalize_angle(&yaw_diff);
            angular_velocity = yaw_diff / dt;
        }
        last_yaw = current_yaw;
        last_yaw_time = current_time;

        // Calculate progress and error
        float progress = fabs(current_yaw - initial_yaw);
        if (progress > 90.0f)
            progress = 90.0f;
        float progress_percent = (progress / 90.0f) * 100.0f;
        float yaw_error = target_yaw - current_yaw;
        normalize_angle(&yaw_error);

        // Check if we should switch to momentum mode
        if (!momentum_mode && progress >= MOMENTUM_THRESHOLD)
        {
            momentum_mode = true;
            log_remote("Switching to momentum mode at %.1f%% progress", progress_percent);
        }

        // Calculate motor speeds based on mode
        unsigned long duty_a, duty_b;
        if (momentum_mode)
        {
            // Predict overshoot based on angular velocity
            float predicted_overshoot = current_yaw + (angular_velocity * 0.5f); // Predict 500ms ahead
            normalize_angle(&predicted_overshoot);

            float overshoot_error = target_yaw - predicted_overshoot;
            normalize_angle(&overshoot_error);

            // If we predict overshooting, apply braking
            if (fabs(overshoot_error) < 0 && fabs(angular_velocity) > ANGULAR_VEL_THRESHOLD)
            {
                braking = true;
                log_remote("Predicted overshoot: %.2f°, Applying brake");
            }

            if (braking)
            {
                // Apply braking force in opposite direction
                duty_a = (unsigned long)(BRAKE_FORCE * PWM_MAX_DUTY);
                duty_b = (unsigned long)(BRAKE_FORCE * PWM_MAX_DUTY);
                set_motor_turn(!turn_right); // Reverse direction for braking
                log_remote("Braking with duty: %lu (%.1f%%)", duty_a, BRAKE_FORCE * 100.0f);
            }
            else
            {
                // Use minimum force to maintain momentum
                duty_a = (unsigned long)(MOMENTUM_SPEED * PWM_MAX_DUTY);
                duty_b = (unsigned long)(MOMENTUM_SPEED * PWM_MAX_DUTY);
                log_remote("Momentum mode with duty: %lu (%.1f%%)", duty_a, MOMENTUM_SPEED * 100.0f);
            }
        }
        else
        {
            // Full power mode until momentum threshold
            duty_a = base_duty;
            duty_b = base_duty;
            log_remote("Full power mode with duty: %lu (%.1f%%)", duty_a, TURN_SPEED * 100.0f);
        }

        // Apply motor speeds
        set_motor_speed(duty_a, duty_b);

        // Log status
        log_remote("Turn progress: %.1f%%, Yaw: %.2f°, Error: %.2f°, Ang. Vel: %.2f°/s, Mode: %s, Duty: %lu",
                   progress_percent, current_yaw, yaw_error, angular_velocity,
                   momentum_mode ? (braking ? "BRAKING" : "MOMENTUM") : "FULL POWER",
                   duty_a);

        // Check for turn completion
        if (fabs(yaw_error) < TURN_TOLERANCE && fabs(angular_velocity) < 2.0f) // Reduced velocity threshold from 5.0f to 2.0f
        {
            if (current_time - start_time >= MIN_TURN_TIME_MS)
            {
                // Apply a stronger final brake to prevent overshooting
                set_motor_turn(!turn_right); // Reverse direction for braking
                set_motor_speed((unsigned long)(BRAKE_FORCE * PWM_MAX_DUTY),
                                (unsigned long)(BRAKE_FORCE * PWM_MAX_DUTY));
                vTaskDelay(pdMS_TO_TICKS(150)); // Increased brake duration from 100ms to 150ms

                turn_control.is_turning = false;
                log_remote("Turn completed successfully");
                break;
            }
        }

        // Check for timeout
        if (current_time - start_time >= TURN_TIMEOUT_MS)
        {
            turn_control.is_turning = false;
            log_remote("Turn timeout reached");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Stop motors
    motor_stop();
    delay_ms(1000);
}

/**
 * @brief Drive forward until 1 meter is traveled.
 *
 * This function resets the encoder counts and then drives forward using a heading
 * PID (via calculate_heading_correction) to keep the vehicle straight.
 * The forward speed is not constant – the speed is reduced as the vehicle approaches
 * the target distance (in centimeters). For a 1-meter drive, call:
 *     motor_forward_distance(0.65f, 100.0f);
 *
 * @param max_speed Maximum speed (as a fraction from 0.0f to 1.0f) to drive at.
 * @param target_distance_cm Target travel distance in centimeters (e.g., 100.0 for 1 meter).
 */
void motor_forward_distance(float max_speed, float target_distance_cm)
{
    // Reset the encoder counts so distance measurement starts at 0.
    encoder_reset();
    // Reset any movement tracking
    mpu_reset_movement();

    // Get the initial heading to maintain throughout the drive.
    target_yaw = mpu_get_orientation().yaw;
    log_remote("Starting drive for %.1f cm at max speed: %.0f%%, target yaw: %.2f°",
               target_distance_cm, max_speed * 100.0f, target_yaw);

    // Set up motors for forward motion.
    set_motor_direction(true);
    // Start with the maximum speed.
    unsigned long base_duty = (unsigned long)(max_speed * PWM_MAX_DUTY);
    set_motor_speed(base_duty, base_duty);

    // Record start time and reset the PID timer.
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long current_time = start_time;
    last_pid_update = start_time;

    // Main drive loop – exit when the traveled distance reaches or exceeds the target.
    while (1)
    {
        // Get the current traveled distance from the encoders (in centimeters)
        float current_distance = encoder_get_distance();
        if (current_distance >= target_distance_cm)
        {
            break;
        }
        // Compute the remaining distance.
        float remaining = target_distance_cm - current_distance;

        // Use a linear deceleration profile starting at 30cm remaining
        float speed = max_speed;
        if (remaining < 30.0f) // Start deceleration at 30cm remaining
        {
            // Use a linear deceleration curve from max_speed to 0.45 * max_speed
            float decel_factor = remaining / 30.0f;
            speed = max_speed * (0.475f + 0.55f * decel_factor);

            // Allow negative speed for braking
            if (speed < 0.0f)
            {
                speed = 0.0f;
            }
        }

        // Get current orientation for heading control.
        mpu_data_t orientation = mpu_get_orientation();
        float current_yaw = orientation.yaw;
        // Calculate heading error (wrap between -180° and +180°).
        float yaw_error = target_yaw - current_yaw;
        while (yaw_error > 180.0f)
            yaw_error -= 360.0f;
        while (yaw_error < -180.0f)
            yaw_error += 360.0f;

        // Update the PID for heading correction.
        unsigned long new_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (new_time - last_pid_update) / 1000.0f;
        if (dt < 0.001f)
        {
            dt = 0.001f; // prevent extremely small dt
        }
        last_pid_update = new_time;
        float correction = calculate_heading_correction(yaw_error, dt);

        // Apply the correction to the motor speeds.
        float duty_ratio_a = speed + correction;
        float duty_ratio_b = speed - correction;

        // Convert ratios to PWM duty cycles.
        long duty_a = (long)(duty_ratio_a * PWM_MAX_DUTY + 0.5f);
        long duty_b = (long)(duty_ratio_b * PWM_MAX_DUTY + 0.5f);
        set_motor_speed(duty_a, duty_b);

        // Log current status.
        log_remote("Distance: %.2f cm, Remaining: %.2f cm, Yaw: %.2f°, Error: %.2f°, Correction: %.2f, Speed: %.0f%%, Duty A: %ld, Duty B: %ld",
                   current_distance, remaining, current_yaw, yaw_error, correction, speed * 100.0f, duty_a, duty_b);

        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to run.
    }

    // Target distance reached; stop the motors.
    log_remote("Target distance reached (%.2f cm). Stopping motors.", encoder_get_distance());
    motor_stop();

    // Continue logging until the car comes to a complete stop
    bool is_moving = true;
    float last_distance = encoder_get_distance();
    unsigned long still_time = 0;
    const unsigned long STILL_THRESHOLD_MS = 1000; // Consider stopped after 1 second of no movement

    while (is_moving)
    {
        float current_distance = encoder_get_distance();
        float distance_change = fabs(current_distance - last_distance);
        mpu_data_t orientation = mpu_get_orientation();
        float current_yaw = orientation.yaw;

        // Log current status
        log_remote("Coasting - Distance: %.2f cm, Yaw: %.2f°, Distance change: %.2f cm",
                   current_distance, current_yaw, distance_change);

        if (distance_change < 0.1f) // Less than 1mm change
        {
            still_time += 10;
            if (still_time >= STILL_THRESHOLD_MS)
            {
                is_moving = false;
                log_remote("Car has come to a complete stop. Final distance: %.2f cm, Final yaw: %.2f°",
                           current_distance, current_yaw);
            }
        }
        else
        {
            still_time = 0;
        }

        last_distance = current_distance;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
