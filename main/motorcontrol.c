// motorcontrol.c

#include "motorcontrol.h"
#include "wifi_logger.h"
#include "mpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION) - 1)

// ============================================================================
// PID Control Parameters
// ============================================================================
#define PID_KP 0.15f       // Proportional gain
#define PID_KI 0.002f      // Integral gain
#define PID_KD 0.01f       // Derivative gain
#define PID_I_MAX 0.2f     // Maximum integral term
#define PID_UPDATE_MS 50   // PID update interval
#define DRIVE_TIME_MS 5000 // Total drive time (10 seconds)

// Add these defines at the top with other defines
#define TURN_PID_KP 0.3f         // Higher proportional gain for faster response
#define TURN_PID_KI 0.01f        // Small integral to handle steady-state error
#define TURN_PID_KD 0.05f        // Derivative to dampen oscillations
#define TURN_PID_I_MAX 0.5f      // Maximum integral term
#define TURN_SPEED 0.5f          // Base speed for turns
#define TURN_TOLERANCE 5.0f      // Increased tolerance for turn completion
#define TURN_TIMEOUT_MS 3000     // Maximum time to complete turn
#define MIN_TURN_TIME_MS 300     // Minimum time to ensure full rotation
#define CALIBRATION_SPEED 0.3f   // Speed for motor calibration
#define CALIBRATION_TIME_MS 2000 // Time to run calibration
#define MIN_MOTOR_FORCE 0.2f     // Minimum force each motor must apply (20% of max)
#define TURN_DEADZONE 1.0f       // Degrees of deadzone to prevent oscillations
#define TURN_MAX_CORRECTION 0.3f // Maximum correction factor (30% of base speed)
#define TURN_SCALE_FACTOR 0.5f   // Scale factor for corrections near target
#define OVERSHOOT_SPEED 0.2f     // Speed when overshooting (40% of normal speed)
#define MOMENTUM_THRESHOLD 70.0f // Degrees at which we start using momentum
#define MOMENTUM_SPEED 0.2f      // Speed when using momentum (40% of normal speed)

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

static void set_motor_speed(unsigned long duty_a, unsigned long duty_b)
{
    // Ensure minimum force on both motors
    unsigned long min_duty = (unsigned long)(MIN_MOTOR_FORCE * PWM_MAX_DUTY);

    // Apply minimum force to both motors
    duty_a = (duty_a < min_duty) ? min_duty : duty_a;
    duty_b = (duty_b < min_duty) ? min_duty : duty_b;

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
    // Update integral with anti-windup
    yaw_integral += error * dt;
    yaw_integral = (yaw_integral > PID_I_MAX) ? PID_I_MAX : (yaw_integral < -PID_I_MAX) ? -PID_I_MAX
                                                                                        : yaw_integral;

    // Calculate derivative
    float derivative = (error - last_yaw_error) / dt;
    last_yaw_error = error;

    // Apply PID control
    return PID_KP * error + PID_KI * yaw_integral + PID_KD * derivative;
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

    // Initialize drive parameters
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long current_time = start_time;
    target_yaw = mpu_get_orientation().yaw; // Get initial heading
    log_remote("Starting straight drive at %.0f%% speed, target yaw: %.2f°", speed * 100, target_yaw);

    // Set up motors for forward motion
    set_motor_direction(true);
    unsigned long base_duty = (unsigned long)(speed * PWM_MAX_DUTY);
    set_motor_speed(base_duty, base_duty);

    // Main drive loop
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
        if (dt >= PID_UPDATE_MS / 1000.0f)
        {
            // Calculate heading correction
            float correction = calculate_heading_correction(yaw_error, dt);

            // Apply correction to motor speeds
            unsigned long duty_a = base_duty + (unsigned long)(correction * PWM_MAX_DUTY);
            unsigned long duty_b = base_duty - (unsigned long)(correction * PWM_MAX_DUTY);

            // Set motor speeds
            set_motor_speed(duty_a, duty_b);

            // Log status periodically
            if ((current_time - start_time) % 1000 < 50) // Log roughly every second
            {
                log_remote("Yaw: %.2f°, Error: %.2f°, Correction: %.2f",
                           current_yaw, yaw_error, correction);
            }

            last_pid_update = new_time;
        }

        // Update time and delay
        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Drive complete
    log_remote("Drive time elapsed, stopping motors...");
    motor_stop();
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
    float target_yaw = initial_yaw + (turn_right ? 90.0f : -90.0f);
    normalize_angle(&target_yaw);

    // Set up motors for turning
    set_motor_turn(turn_right);
    unsigned long base_duty = (unsigned long)(TURN_SPEED * PWM_MAX_DUTY);
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
        // Get current orientation
        mpu_data_t orientation = mpu_get_orientation();
        float current_yaw = orientation.yaw;
        float yaw_error = target_yaw - current_yaw;
        normalize_angle(&yaw_error);

        // Calculate progress percentage
        float progress = fabs(current_yaw - initial_yaw);
        if (progress > 90.0f)
            progress = 90.0f;
        float progress_percent = (progress / 90.0f) * 100.0f;

        // Update PID control
        unsigned long current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (current_time - turn_control.last_update) / 1000.0f;
        if (dt >= PID_UPDATE_MS / 1000.0f)
        {
            // Calculate correction
            float correction = calculate_turn_correction(yaw_error, dt);

            // Apply correction while maintaining minimum force
            unsigned long duty_a = base_duty + (unsigned long)(correction * PWM_MAX_DUTY);
            unsigned long duty_b = base_duty - (unsigned long)(correction * PWM_MAX_DUTY);
            set_motor_speed(duty_a, duty_b);

            // Log status
            log_remote("Turn progress: %.1f%%, Yaw: %.2f°, Error: %.2f°, Correction: %.2f",
                       progress_percent, current_yaw, yaw_error, correction);

            turn_control.last_update = current_time;
        }

        // Check for turn completion
        if (fabs(yaw_error) < TURN_TOLERANCE)
        {
            // Ensure minimum time has elapsed
            if (current_time - start_time >= MIN_TURN_TIME_MS)
            {
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
}
