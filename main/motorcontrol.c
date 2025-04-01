// motorcontrol.c

#include "motorcontrol.h"
#include "wifi_logger.h"
#include "mpu.h"
#include "ultrasonic.h"
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
// Motor A pins (Right motor)
#define MOTOR_A_IN1 37 // Direction pin 1
#define MOTOR_A_IN2 48 // Direction pin 2
#define MOTOR_A_EN 38  // Enable pin (PWM)

// Motor B pins (Left motor)
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
#define PID_KP 0.15f       // Reduced from 0.25f for gentler response
#define PID_KI 0.001f      // Reduced from 0.003f to prevent integral windup
#define PID_KD 0.02f       // Reduced from 0.05f to prevent derivative spikes
#define PID_I_MAX 0.2f     // Reduced from 0.3f to limit integral action
#define PID_UPDATE_MS 50   // PID update interval
#define DRIVE_TIME_MS 5000 // Total drive time (10 seconds)

// Add these defines at the top with other defines
#define TURN_PID_KP 0.3f            // Reduced from 0.4f to 0.3f for smoother control
#define TURN_PID_KI 0.01f           // Reduced from 0.02f to 0.01f
#define TURN_PID_KD 0.1f            // Increased from 0.08f to 0.1f for better damping
#define TURN_PID_I_MAX 0.5f         // Reduced from 0.8f to 0.5f
#define TURN_SPEED 0.7f             // Reduced from 0.8f to 0.7f (70% power)
#define TURN_TOLERANCE 5.0f         // Increased from 2.0f to 5.0f for more stable turns
#define TURN_TIMEOUT_MS 5000        // Increased from 3000 to 5000 (5 seconds)
#define MIN_TURN_TIME_MS 300        // Minimum time to ensure full rotation
#define CALIBRATION_SPEED 0.3f      // Speed for motor calibration
#define CALIBRATION_TIME_MS 2000    // Time to run calibration
#define MIN_MOTOR_FORCE 0.5f        // Increased from 0.4f to 0.5f (50% power)
#define MIN_MOTOR_SPEED 0.2f        // Minimum motor speed (20% power)
#define OVERSTEER_THRESHOLD 6.0f    // Oversteer detection threshold (6 degrees)
#define TURN_DEADZONE 1.0f          // Degrees of deadzone to prevent oscillations
#define TURN_MAX_CORRECTION 0.3f    // Maximum correction factor (30% of base speed)
#define TURN_SCALE_FACTOR 0.5f      // Scale factor for corrections near target
#define MOMENTUM_THRESHOLD 35.0f    // Reduced from 45.0f to 35.0f to start slowing down earlier
#define MOMENTUM_SPEED 0.3f         // Reduced from 0.4f to 0.3f (30% power)
#define ANGULAR_VEL_THRESHOLD 40.0f // Reduced from 50.0f to 40.0f to start braking earlier
#define BRAKE_FORCE 0.6f            // Increased from 0.4f to 0.6f (60% power)
#define TARGET_DISTANCE_CM 15.0f    // Changed from 11.0f to 15.0f
#define DISTANCE_TOLERANCE_CM 1.0f  // Allowed error in distance
#define DISTANCE_PID_KP 0.3f        // Proportional gain for distance control
#define DISTANCE_PID_KI 0.01f       // Integral gain for distance control
#define DISTANCE_PID_KD 0.05f       // Derivative gain for distance control
#define DISTANCE_PID_I_MAX 0.5f     // Maximum integral term
#define DISTANCE_UPDATE_MS 50       // PID update interval
#define MIN_DISTANCE_CM 5.0f        // Minimum safe distance
#define MAX_DISTANCE_CM 100.0f      // Maximum distance to consider

// Left turn parameters
#define LEFT_TURN_SPEED 0.6f             // Reduced from 0.8f to 0.7f for safer turns
#define LEFT_TURN_PID_KP 0.35f           // Increased from 0.25f to compensate for removed integral
#define LEFT_TURN_PID_KI 0.0f            // Removed integral term
#define LEFT_TURN_PID_KD 0.08f           // Increased from 0.05f for better damping
#define LEFT_TURN_PID_I_MAX 0.0f         // Removed integral max since we're not using integral
#define LEFT_MOMENTUM_THRESHOLD 45.0f    // Later momentum mode for left turns
#define LEFT_MOMENTUM_SPEED 0.4f         // Higher momentum speed for left turns
#define LEFT_ANGULAR_VEL_THRESHOLD 50.0f // Higher velocity threshold for left turns

// Right turn parameters
#define RIGHT_TURN_SPEED 0.6f             // Lower speed for right turns
#define RIGHT_TURN_PID_KP 0.35f           // Increased from 0.25f to compensate for removed integral
#define RIGHT_TURN_PID_KI 0.0f            // Removed integral term
#define RIGHT_TURN_PID_KD 0.08f           // Increased from 0.15f for better damping
#define RIGHT_TURN_PID_I_MAX 0.0f         // Removed integral max since we're not using integral
#define RIGHT_MOMENTUM_THRESHOLD 35.0f    // Earlier momentum mode for right turns
#define RIGHT_MOMENTUM_SPEED 0.3f         // Lower momentum speed for right turns
#define RIGHT_ANGULAR_VEL_THRESHOLD 40.0f // Lower velocity threshold for right turns

// Add these defines at the top with other defines
#define FORWARD_SPEED 0.7f         // Increased from 0.6f to 0.8f (80% power)
#define MAX_FORWARD_SPEED 0.8f     // Increased from 0.6f to 0.8f (80% power)
#define BRAKE_START_DISTANCE 45.0f // Start braking from further away
#define BRAKE_END_DISTANCE 11.0f   // Changed from 15.0f to 11.0f
#define BRAKE_PID_KP 0.08f         // Very gentle proportional braking
#define BRAKE_PID_KI 0.001f        // Minimal integral to prevent oscillation
#define BRAKE_PID_KD 0.01f         // Small derivative for smoothness
#define BRAKE_TIME_MS 100          // Very short final brake
#define MIN_STOP_DISTANCE 11.0f    // Changed from 15.0f to 11.0f
#define EMERGENCY_BRAKE_FORCE 0.3f // Reduced emergency brake force
#define MAX_BRAKE_FORCE 0.2f       // Maximum 20% power for braking
#define MIN_BRAKE_FORCE 0.05f      // Minimum 5% power for braking

// Add deadzone for forward driving
#define FORWARD_DEADZONE 2.0f       // Degrees of deadzone to prevent small corrections
#define FORWARD_MAX_CORRECTION 0.3f // Increased from 0.2f to allow stronger corrections

// Add new path detection parameters
#define PATH_DETECTION_DISTANCE 30.0f // Consider path available if distance > 30cm

// Add these defines for better heading control
#define MAX_DERIVATIVE 100.0f  // Maximum allowed derivative value
#define MAX_CORRECTION 0.2f    // Maximum allowed correction (20% of base speed)
#define HEADING_DEADZONE 5.0f  // Increased deadzone to prevent small corrections
#define HEADING_SMOOTHING 0.3f // Smoothing factor for derivative calculation

// Add these defines for ultrasonic validation
#define MIN_VALID_DISTANCE 5.0f     // Minimum valid distance reading
#define MAX_VALID_DISTANCE 400.0f   // Maximum valid distance reading
#define MAX_DISTANCE_CHANGE 100.0f  // Maximum allowed distance change per second
#define INVALID_READING_THRESHOLD 3 // Number of invalid readings before taking action

// Add these defines at the top with other defines
typedef enum
{
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_BOTH
} motor_select_t;

// Add this new enum for motor direction
typedef enum
{
    MOTOR_FORWARD = 1,
    MOTOR_BACKWARD = -1,
    MOTOR_STOP = 0
} motor_direction_t;

// Add after the other type definitions (after motor_direction_t enum)
static turn_pid_t left_turn_pid = {
    .integral = 0.0f,
    .last_error = 0.0f,
    .kp = LEFT_TURN_PID_KP,
    .ki = LEFT_TURN_PID_KI,
    .kd = LEFT_TURN_PID_KD,
    .i_max = LEFT_TURN_PID_I_MAX};

static turn_pid_t right_turn_pid = {
    .integral = 0.0f,
    .last_error = 0.0f,
    .kp = RIGHT_TURN_PID_KP,
    .ki = RIGHT_TURN_PID_KI,
    .kd = RIGHT_TURN_PID_KD,
    .i_max = RIGHT_TURN_PID_I_MAX};

// Add after the turn_pid_t struct definition and before the helper functions
static float calculate_turn_correction(float error, float dt, turn_pid_t *pid);
void apply_brake_burst(motor_direction_t left_dir, motor_direction_t right_dir, float current_speed);

// ============================================================================
// State Variables
// ============================================================================
static bool is_braking = false;
static float current_speed = 0.0f;
static float target_yaw = 0.0f;
static float yaw_integral = 0.0f;
static float last_yaw_error = 0.0f;
static unsigned long last_pid_update = 0;
static float distance_integral = 0.0f;
static float last_distance_error = 0.0f;
static unsigned long last_distance_update = 0;

// Add these global variables at the top with other state variables
turn_control_t turn_control = {0};
motor_calibration_t motor_calibration = {
    .left_factor = 1.0f,
    .right_factor = 1.0f,
    .is_calibrated = false};

// Add these declarations at the top with other state variables
static bool is_forward_movement = false;
static bool was_stopped = true;
static unsigned long boost_start_time = 0;

// ============================================================================
// Helper Functions
// ============================================================================
static void set_motors(float left_pwm, motor_direction_t left_dir, float right_pwm, motor_direction_t right_dir)
{
    // Input validation
    left_pwm = (left_pwm < 0.0f) ? 0.0f : (left_pwm > 1.0f) ? 1.0f
                                                            : left_pwm;
    right_pwm = (right_pwm < 0.0f) ? 0.0f : (right_pwm > 1.0f) ? 1.0f
                                                               : right_pwm;

    // Convert PWM values to duty cycles
    unsigned long left_duty = (unsigned long)(left_pwm * PWM_MAX_DUTY);
    unsigned long right_duty = (unsigned long)(right_pwm * PWM_MAX_DUTY);

    // Apply minimum force if either motor is moving
    if (left_dir != MOTOR_STOP || right_dir != MOTOR_STOP)
    {
        unsigned long min_duty = (unsigned long)(MIN_MOTOR_FORCE * PWM_MAX_DUTY);
        if (left_dir != MOTOR_STOP)
            left_duty = (left_duty < min_duty) ? min_duty : left_duty;
        if (right_dir != MOTOR_STOP)
            right_duty = (right_duty < min_duty) ? min_duty : right_duty;
    }

    // Set left motor direction and PWM (Motor B)
    gpio_set_level(MOTOR_B_IN1, left_dir == MOTOR_FORWARD ? 1 : 0);
    gpio_set_level(MOTOR_B_IN2, left_dir == MOTOR_BACKWARD ? 1 : 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, left_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);

    // Set right motor direction and PWM (Motor A)
    gpio_set_level(MOTOR_A_IN1, right_dir == MOTOR_FORWARD ? 1 : 0);
    gpio_set_level(MOTOR_A_IN2, right_dir == MOTOR_BACKWARD ? 1 : 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, right_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

    // Update forward movement mode based on motor directions
    bool new_forward_movement = (left_dir == MOTOR_FORWARD && right_dir == MOTOR_FORWARD);
    if (is_forward_movement != new_forward_movement)
    {
        is_forward_movement = new_forward_movement;
        if (!is_forward_movement)
        {
            // Reset boost state when leaving forward movement
            was_stopped = true;
            boost_start_time = 0;
        }
    }
}

static void set_motor_direction(bool forward)
{
    motor_direction_t dir = forward ? MOTOR_FORWARD : MOTOR_BACKWARD;
    set_motors(0.0f, dir, 0.0f, dir);
}

static void set_motor_turn(bool turn_right)
{
    if (turn_right)
    {
        set_motors(0.0f, MOTOR_BACKWARD, 0.0f, MOTOR_FORWARD);
    }
    else
    {
        set_motors(0.0f, MOTOR_FORWARD, 0.0f, MOTOR_BACKWARD);
    }
}

static void set_motor_speed(unsigned long duty_a, unsigned long duty_b)
{
    float left_pwm = (float)duty_b / PWM_MAX_DUTY;
    float right_pwm = (float)duty_a / PWM_MAX_DUTY;
    set_motors(left_pwm, MOTOR_FORWARD, right_pwm, MOTOR_FORWARD);
}

static float calculate_heading_correction(float error, float dt)
{
    // Apply deadzone to prevent small corrections
    if (fabs(error) < HEADING_DEADZONE)
    {
        error = 0;
        yaw_integral = 0; // Reset integral in deadzone
    }

    // Update integral with anti-windup
    yaw_integral += error * dt;
    yaw_integral = (yaw_integral > PID_I_MAX) ? PID_I_MAX : (yaw_integral < -PID_I_MAX) ? -PID_I_MAX
                                                                                        : yaw_integral;

    // Calculate derivative with low-pass filter and limit maximum value
    float derivative = (error - last_yaw_error) / dt;
    derivative = derivative > MAX_DERIVATIVE ? MAX_DERIVATIVE : derivative < -MAX_DERIVATIVE ? -MAX_DERIVATIVE
                                                                                             : derivative;

    // Apply smoothing to derivative
    static float smoothed_derivative = 0;
    smoothed_derivative = HEADING_SMOOTHING * derivative +
                          (1.0f - HEADING_SMOOTHING) * smoothed_derivative;

    last_yaw_error = error;

    // Apply PID control with reduced gains
    float correction = (PID_KP * error + PID_KI * yaw_integral + PID_KD * smoothed_derivative) * 0.5f;

    // Limit maximum correction
    if (correction > MAX_CORRECTION)
    {
        correction = MAX_CORRECTION;
    }
    else if (correction < -MAX_CORRECTION)
    {
        correction = -MAX_CORRECTION;
    }

    // Log PID components for debugging
    static unsigned long last_pid_log = 0;
    unsigned long current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_pid_log >= 200) // Log every 200ms
    {
        log_remote("PID Components - Error: %.2f°, P: %.2f, I: %.2f, D: %.2f, Final: %.2f",
                   error, PID_KP * error, PID_KI * yaw_integral, PID_KD * smoothed_derivative, correction);
        last_pid_log = current_time;
    }

    return correction;
}

static float calculate_turn_correction(float error, float dt, turn_pid_t *pid)
{
    // Calculate derivative
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    // Apply deadzone to prevent small oscillations
    if (fabs(error) < TURN_DEADZONE)
    {
        error = 0;
    }

    // Calculate base correction (removed integral term)
    float correction = pid->kp * error + pid->kd * derivative;

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

static float calculate_distance_correction(float error, float dt)
{
    // Update integral with anti-windup
    distance_integral += error * dt;
    distance_integral = (distance_integral > DISTANCE_PID_I_MAX) ? DISTANCE_PID_I_MAX : (distance_integral < -DISTANCE_PID_I_MAX) ? -DISTANCE_PID_I_MAX
                                                                                                                                  : distance_integral;

    // Calculate derivative
    float derivative = (error - last_distance_error) / dt;
    last_distance_error = error;

    // Apply PID control
    return DISTANCE_PID_KP * error + DISTANCE_PID_KI * distance_integral + DISTANCE_PID_KD * derivative;
}

// Remove static from the function definition to make it public
bool check_and_turn_available_path(void)
{
    log_remote("Taking ultrasonic measurements for 2 seconds...");

    // Variables to store sum of readings
    float left_sum = 0;
    float right_sum = 0;
    int reading_count = 0;

    // Take readings for 2 seconds
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < 2000)
    {
        ultrasonic_readings_t readings = ultrasonic_get_all();
        // Remember: left and right sensors are swapped in hardware
        left_sum += readings.right; // right sensor reads left side
        right_sum += readings.left; // left sensor reads right side
        reading_count++;
        vTaskDelay(pdMS_TO_TICKS(50)); // Take a reading every 50ms
    }

    // Calculate averages
    float left_avg = left_sum / reading_count;
    float right_avg = right_sum / reading_count;

    log_remote("Average readings over %d samples - Left: %.1fcm, Right: %.1fcm",
               reading_count, left_avg, right_avg);

    // Check for available paths using averaged readings
    bool right_available = right_avg > PATH_DETECTION_DISTANCE;
    bool left_available = left_avg > PATH_DETECTION_DISTANCE;

    // If both paths available, prefer right
    if (right_available)
    {
        log_remote("Right path available (avg %.1fcm), turning right", right_avg);
        motor_turn_90(true); // true for right turn
        return true;
    }
    else if (left_available)
    {
        log_remote("Only left path available (avg %.1fcm), turning left", left_avg);
        motor_turn_90(false); // false for left turn
        return true;
    }

    log_remote("No available paths detected");
    return false;
}

// Add this new function before the public interface functions
static void set_motor_pwm(float left_pwm, float right_pwm, bool forward)
{
    // Convert PWM values (0.0 to 1.0) to duty cycles
    unsigned long left_duty = (unsigned long)(left_pwm * PWM_MAX_DUTY);
    unsigned long right_duty = (unsigned long)(right_pwm * PWM_MAX_DUTY);

    // Set left motor direction (Motor B)
    gpio_set_level(MOTOR_B_IN1, forward ? 1 : 0);
    gpio_set_level(MOTOR_B_IN2, forward ? 0 : 1);
    // Set left motor PWM
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, left_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);

    // Set right motor direction (Motor A)
    gpio_set_level(MOTOR_A_IN1, forward ? 1 : 0);
    gpio_set_level(MOTOR_A_IN2, forward ? 0 : 1);
    // Set right motor PWM
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, right_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
}

// Add this helper function implementation
void apply_brake_burst(motor_direction_t left_dir, motor_direction_t right_dir, float current_speed)
{
    // Store current motor directions
    motor_direction_t brake_left_dir = (left_dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
    motor_direction_t brake_right_dir = (right_dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;

    // Apply gentler brake burst
    set_motors(0.45f, brake_left_dir, 0.45f, brake_right_dir);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Return to normal speed and direction
    set_motors(current_speed, left_dir, current_speed, right_dir);
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
    set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);

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
    set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);
    log_remote("Motor stop complete");
}

void motor_turn_90(bool turn_right)
{
    // Reset forward movement mode
    is_forward_movement = false;
    was_stopped = true;
    boost_start_time = 0;

    // Initialize turn parameters
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float initial_yaw = mpu_get_orientation().yaw;

    // Calculate target yaw based on relative turn
    float target_yaw = initial_yaw + (turn_right ? -90.0f : 90.0f);
    normalize_angle(&target_yaw);

    // Calculate the shortest path to the target
    float yaw_diff = target_yaw - initial_yaw;
    normalize_angle(&yaw_diff);

    // If the difference is greater than 180 degrees, we should turn in the opposite direction
    if (fabs(yaw_diff) > 180.0f)
    {
        turn_right = !turn_right; // Reverse the turn direction
        target_yaw = initial_yaw + (turn_right ? -90.0f : 90.0f);
        normalize_angle(&target_yaw);
        log_remote("Taking shorter path by reversing turn direction");
    }

    // Set initial motor speeds and directions
    float turn_speed = turn_right ? RIGHT_TURN_SPEED : LEFT_TURN_SPEED;
    motor_direction_t left_dir = turn_right ? MOTOR_FORWARD : MOTOR_BACKWARD;
    motor_direction_t right_dir = turn_right ? MOTOR_BACKWARD : MOTOR_FORWARD;

    // Log initial setup
    log_remote("Turn Setup - Initial Yaw: %.2f°, Target Yaw: %.2f°, Turn: %s",
               initial_yaw, target_yaw, turn_right ? "right" : "left");
    log_remote("Motor Configuration:");
    log_remote("  Left motor (B): %.1f%% %s", turn_speed * 100.0f,
               left_dir == MOTOR_FORWARD ? "forward" : "backward");
    log_remote("  Right motor (A): %.1f%% %s", turn_speed * 100.0f,
               right_dir == MOTOR_FORWARD ? "forward" : "backward");

    set_motors(turn_speed, left_dir, turn_speed, right_dir);

    // Main turn loop
    while (true)
    {
        // Get current orientation and time
        mpu_data_t orientation = mpu_get_orientation();
        float current_yaw = orientation.yaw;
        unsigned long current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Calculate progress based on actual rotation
        float progress;
        if (turn_right)
        {
            // For right turns, progress is positive when yaw decreases
            progress = initial_yaw - current_yaw;
        }
        else
        {
            // For left turns, progress is positive when yaw increases
            // Handle the case when crossing 0/360 boundary
            if (initial_yaw > 270.0f && current_yaw < 90.0f)
            {
                // We've crossed the 0/360 boundary clockwise
                progress = (360.0f - initial_yaw) + current_yaw;
            }
            else if (current_yaw < initial_yaw)
            {
                // We've crossed the 0/360 boundary counterclockwise
                progress = (360.0f - initial_yaw) + current_yaw;
            }
            else
            {
                progress = current_yaw - initial_yaw;
            }
        }
        normalize_angle(&progress);

        // Ensure progress is positive
        if (progress < 0)
        {
            progress += 360.0f;
        }

        // Cap progress at 90 degrees
        if (progress > 90.0f)
        {
            progress = 90.0f;
        }

        float progress_percent = (progress / 90.0f) * 100.0f;

        // Calculate error taking shortest path
        float yaw_error = target_yaw - current_yaw;
        normalize_angle(&yaw_error);

        // Calculate time delta for PID
        static unsigned long last_time = 0;
        float dt = (current_time - last_time) / 1000.0f; // Convert to seconds
        last_time = current_time;

        // Calculate PID correction based on turn direction
        float correction;
        if (turn_right)
        {
            correction = calculate_turn_correction(yaw_error, dt, &right_turn_pid);
            log_remote("Right turn PID - Error: %.2f°, Correction: %.2f", yaw_error, correction);
        }
        else
        {
            correction = calculate_turn_correction(yaw_error, dt, &left_turn_pid);
            log_remote("Left turn PID - Error: %.2f°, Correction: %.2f", yaw_error, correction);
        }

        // Apply correction to motor speeds
        float current_speed = turn_speed;
        motor_direction_t current_left_dir = left_dir;
        motor_direction_t current_right_dir = right_dir;

        // Scale speed based on error magnitude
        float error_magnitude = fabs(yaw_error);
        float speed_scale = 1.0f;

        // Start reducing speed when within 45 degrees of target
        if (error_magnitude < 45.0f)
        {
            speed_scale = error_magnitude / 45.0f;
            // Ensure minimum speed to maintain movement
            speed_scale = (speed_scale < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : speed_scale;
        }

        // Apply base speed with scaling
        current_speed = turn_speed * speed_scale;

        // Handle oversteering by reversing motor directions if needed
        if (turn_right)
        {
            // For right turns, error starts negative and becomes positive when oversteering
            if (yaw_error > OVERSTEER_THRESHOLD) // Only detect oversteer when exceeding 6 degrees
            {
                // We've oversteered (error became positive)
                current_left_dir = (left_dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
                current_right_dir = (right_dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
                log_remote("Right turn oversteer detected - Error: %.2f°", yaw_error);

                // Apply a short boost to ensure movement
                set_motors(0.4f, current_left_dir, 0.4f, current_right_dir);
                vTaskDelay(pdMS_TO_TICKS(100)); // Boost for 100ms

                // Return to normal speed
                set_motors(current_speed, current_left_dir, current_speed, current_right_dir);
            }
        }
        else
        {
            // For left turns, error starts positive and becomes negative when oversteering
            if (yaw_error < -OVERSTEER_THRESHOLD) // Only detect oversteer when exceeding -6 degrees
            {
                // We've oversteered (error became negative)
                current_left_dir = (left_dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
                current_right_dir = (right_dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
                log_remote("Left turn oversteer detected - Error: %.2f°", yaw_error);

                // Apply a short boost to ensure movement
                set_motors(0.4f, current_left_dir, 0.4f, current_right_dir);
                vTaskDelay(pdMS_TO_TICKS(100)); // Boost for 100ms

                // Return to normal speed
                set_motors(current_speed, current_left_dir, current_speed, current_right_dir);
            }
        }

        // Apply current motor settings
        set_motors(current_speed, current_left_dir, current_speed, current_right_dir);

        // Log status every 100ms
        static unsigned long last_log_time = 0;
        if (current_time - last_log_time >= 100)
        {
            log_remote("Turn Status - Yaw: %.2f°, Progress: %.1f%%, Error: %.2f°, Correction: %.2f",
                       current_yaw, progress_percent, yaw_error, correction);
            log_remote("Motor Status - Left: %.1f%% %s, Right: %.1f%% %s",
                       current_speed * 100.0f, current_left_dir == MOTOR_FORWARD ? "forward" : "backward",
                       current_speed * 100.0f, current_right_dir == MOTOR_FORWARD ? "forward" : "backward");
            last_log_time = current_time;
        }

        // Check for turn completion
        static unsigned long stable_start_time = 0;
        static bool was_stable = false;
        static bool brake_applied = false; // Track if we've applied brake in this stability period

        if (fabs(yaw_error) < TURN_TOLERANCE)
        {
            if (!was_stable)
            {
                stable_start_time = current_time;
                was_stable = true;
                brake_applied = false; // Reset brake flag when entering stability zone
                log_remote("Turn entered stable region (within 5°). Starting 2-second stability check...");

                // Stop motors completely when entering stability zone
                set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);
                log_remote("Motors stopped in stability zone");
            }
            else if (current_time - stable_start_time >= 2000) // 2 seconds of stability
            {
                log_remote("Turn completed successfully - Stable for 2 seconds");
                log_remote("Final Yaw: %.2f°, Target Yaw: %.2f°, Error: %.2f°",
                           current_yaw, target_yaw, yaw_error);
                break;
            }
        }
        else
        {
            if (was_stable)
            {
                log_remote("Turn lost stability - Error: %.2f°. Resetting stability timer.", yaw_error);
                was_stable = false;
                stable_start_time = 0;
                brake_applied = false; // Reset brake flag when losing stability

                // Restart motors with current speed and direction
                set_motors(current_speed, current_left_dir, current_speed, current_right_dir);
                log_remote("Motors restarted after losing stability");
            }
        }

        // Apply brake burst when entering stability zone
        static unsigned long last_brake_time = 0;
        if (fabs(yaw_error) < TURN_TOLERANCE && !brake_applied &&
            (current_time - last_brake_time) >= 500) // Minimum 500ms between brake bursts
        {
            log_remote("Applying brake burst at stability entry - Error: %.2f°", yaw_error);
            apply_brake_burst(current_left_dir, current_right_dir, current_speed);
            brake_applied = true;
            last_brake_time = current_time;

            // Stop motors after brake burst
            set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);
            log_remote("Motors stopped after brake burst");
        }

        // Check for timeout
        if (current_time - start_time >= TURN_TIMEOUT_MS)
        {
            log_remote("Turn timeout reached");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Stop motors
    motor_stop();
}

// Update the test function to use the new unified interface
void test_motors_sequence(void)
{
    log_remote("Starting motor test sequence...");

    // First stop all motors
    set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Test left motor first
    log_remote("Testing left motor for 2 seconds...");
    set_motors(0.7f, MOTOR_FORWARD, 0.0f, MOTOR_STOP);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Stop all motors
    set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Test right motor second
    log_remote("Testing right motor for 2 seconds...");
    set_motors(0.0f, MOTOR_STOP, 0.7f, MOTOR_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Stop all motors
    set_motors(0.0f, MOTOR_STOP, 0.0f, MOTOR_STOP);

    log_remote("Motor test sequence complete");
}
