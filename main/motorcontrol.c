#include "exploration_algorithm/direction.h"  // Provides the Direction enum: NORTH, EAST, SOUTH, WEST, etc.
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
#include "esp_log.h"
#include "esp_timer.h"

#define BLOCK_SIZE 40.0f

// ============================================================================
// Hardware Configuration
// ============================================================================
// Motor A pins (Right motor)
#define MOTOR_A_IN1 37  // Input 1
#define MOTOR_A_IN2 48  // Input 2
#define MOTOR_A_EN 38   // Enable 1,2

// Motor B pins (Left motor)
#define MOTOR_B_IN1 35  // Input 3
#define MOTOR_B_IN2 36  // Input 4
#define MOTOR_B_EN 45   // Enable 3,4

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
#define PID_KP 0.015f         // Further reduced for gentler response
#define PID_KI 0.0002f       // Reduced integral action
#define PID_KD 0.00135f      // Increased derivative gain for better damping
#define PID_I_MAX 0.03f      // Reduced integral windup limit
#define PID_UPDATE_MS 10     // Reduced update interval for more responsive control
#define DRIVE_TIME_MS 5000   // Total drive time (10 seconds)
#define PID_DEADZONE 0.25f   // Reduced deadzone to 0.5 degrees for more precise control

// ============================================================================
// Turn Control Parameters
// ============================================================================
#define TURN_SPEED 0.65f     // Speed for turning
#define TURN_TOLERANCE 2.0f  // Tolerance for turn completion
#define MIN_TURN_TIME_MS 300 // Minimum time for a turn

// Additional parameters for turning and calibration (not used directly in this function)
#define TURN_PID_KP 0.3f            
#define TURN_PID_KI 0.01f          
#define TURN_PID_KD 0.05f         
#define TURN_PID_I_MAX 0.5f       
#define CALIBRATION_SPEED 0.3f      
#define CALIBRATION_TIME_MS 2000    
#define MIN_MOTOR_FORCE 0.4f        
#define TURN_DEADZONE 6.0f           
#define TURN_MAX_CORRECTION 0.3f    
#define TURN_SCALE_FACTOR 0.5f      
#define MOMENTUM_THRESHOLD 45.0f    
#define MOMENTUM_SPEED 0.3f         
#define ANGULAR_VEL_THRESHOLD 50.0f 
#define BRAKE_FORCE 0.6f            
#define MAX_DERIVATIVE 100.0f       

// ============================================================================
// State Variables
// ============================================================================
static bool is_braking = false;
static float current_speed = 0.0f;

static float yaw_integral = 0.0f;
static float last_yaw_error = 0.0f;
static unsigned long last_pid_update = 0;

turn_control_t turn_control = {0};
motor_calibration_t motor_calibration = {
    .left_factor = 1.0f,
    .right_factor = 1.0f,
    .is_calibrated = false
};

// ============================================================================
// Helper Functions
// ============================================================================

// Normalize an angle to the range [0, 360) degrees.
static float normalize_angle(float angle)
{
    while (angle >= 360.0f)
        angle -= 360.0f;
    while (angle < 0.0f)
        angle += 360.0f;
    return angle;
}

// Calculate the minimal heading error given a target and current angle.
// Both angles are normalized to [0, 360) before computing the error.
// The error is then adjusted to be the smallest signed angle difference.
static float calculate_yaw_error(float target_yaw, float current_yaw) {
    target_yaw = normalize_angle(target_yaw);
    current_yaw = normalize_angle(current_yaw);
    float error = target_yaw - current_yaw;
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;
    return error;
}

// Compute the deceleration-adjusted speed based on remaining distance.
static float compute_decelerated_speed(float motor_speed, float remaining, float deceleration_threshold_cm) {
    if (remaining < deceleration_threshold_cm) {
        float decel_factor = remaining / deceleration_threshold_cm;
        return motor_speed * (0.475f + 0.525f * decel_factor);
    }
    return motor_speed;
}

// Check if the vehicle is effectively stationary based on encoder difference.
static bool is_stationary(float last_distance, float current_distance, float threshold) {
    return (fabs(current_distance - last_distance) < threshold);
}

// Wait for the vehicle to come to a complete stop and log coasting status.
static void wait_for_stop(unsigned long still_threshold_ms, drive_result_t *result, float target_yaw) {
    float coasting_last_distance = encoder_get_distance();
    unsigned long coasting_still_time = 0;
    bool is_moving = true;
    
    while (is_moving) {
        float current_distance = encoder_get_distance();
        float distance_change = fabs(current_distance - coasting_last_distance);
        mpu_data_t orientation = mpu_get_orientation();
        
        // Only log yaw-related information
        log_remote("Coasting - Yaw: %.2f°", orientation.yaw);
        
        if (distance_change < 0.1f) {  
            coasting_still_time += 10;
            if (coasting_still_time >= still_threshold_ms) {
                is_moving = false;
                float final_yaw_error = calculate_yaw_error(target_yaw, orientation.yaw);
                result->heading = final_yaw_error;
                result->ultrasonic = ultrasonic_get_all();
                // Only log yaw-related information
                log_remote("Car has come to a complete stop. Final yaw: %.2f° (error: %.2f°)",
                           orientation.yaw, final_yaw_error);
            }
        } else {
            coasting_still_time = 0;
        }
        coasting_last_distance = current_distance;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void set_motor_direction(bool forward)
{
    if (forward)
    {
        gpio_set_level(MOTOR_A_IN1, 1);
        gpio_set_level(MOTOR_A_IN2, 0);
        gpio_set_level(MOTOR_B_IN1, 1);
        gpio_set_level(MOTOR_B_IN2, 0);
    }
    else
    {
        gpio_set_level(MOTOR_A_IN1, 0);
        gpio_set_level(MOTOR_A_IN2, 1);
        gpio_set_level(MOTOR_B_IN1, 0);
        gpio_set_level(MOTOR_B_IN2, 1);
    }
}

static void set_motor_speed(long duty_a, long duty_b)
{
    bool motor_a_forward = true;
    bool motor_b_forward = true;

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

    gpio_set_level(MOTOR_A_IN1, motor_a_forward ? 1 : 0);
    gpio_set_level(MOTOR_A_IN2, motor_a_forward ? 0 : 1);
    gpio_set_level(MOTOR_B_IN1, motor_b_forward ? 1 : 0);
    gpio_set_level(MOTOR_B_IN2, motor_b_forward ? 0 : 1);

    duty_a = (duty_a > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty_a;
    duty_b = (duty_b > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty_b;

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, duty_a);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, duty_b);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);

    log_remote("Motor speeds set - Left: %ld (%.1f%%, %s), Right: %ld (%.1f%%, %s)", 
               duty_a, (float)duty_a/PWM_MAX_DUTY*100.0f, motor_a_forward ? "FORWARD" : "BACKWARD",
               duty_b, (float)duty_b/PWM_MAX_DUTY*100.0f, motor_b_forward ? "FORWARD" : "BACKWARD");
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
    if (dt < 0.001f)
        dt = 0.001f;

    if (fabs(error) < PID_DEADZONE)
    {
        error = 0;
        yaw_integral = 0;
    }

    yaw_integral += error * dt;
    if (yaw_integral > PID_I_MAX)
        yaw_integral = PID_I_MAX;
    else if (yaw_integral < -PID_I_MAX)
        yaw_integral = -PID_I_MAX;

    float derivative = (error - last_yaw_error) / dt;
    if (derivative > MAX_DERIVATIVE)
        derivative = MAX_DERIVATIVE;
    else if (derivative < -MAX_DERIVATIVE)
        derivative = -MAX_DERIVATIVE;

    float output = PID_KP * error + PID_KI * yaw_integral + PID_KD * derivative;
    last_yaw_error = error;
    return output;
}

// ============================================================================
// Public Interface Functions
// ============================================================================

void motor_init(void)
{
    // log_remote("Initializing motor driver...");
    
    gpio_config_t dir_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&dir_conf);
    
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    ledc_channel_config_t channel_conf = {
        .gpio_num = MOTOR_A_EN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_A,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
    channel_conf.gpio_num = MOTOR_B_EN;
    channel_conf.channel = PWM_CHANNEL_B;
    ledc_channel_config(&channel_conf);
    
    current_speed = 0.0f;
    is_braking = false;
    yaw_integral = 0.0f;
    last_yaw_error = 0.0f;
    last_pid_update = 0;
    stop_motors();
    
    // log_remote("Motor driver initialization complete");
}

void motor_stop(void)
{
    current_speed = 0.0f;
    is_braking = false;
    yaw_integral = 0.0f;
    last_yaw_error = 0.0f;
    stop_motors();
    // log_remote("Motor stop complete");
}

/**
 * @brief Drive forward for a specific distance with compensation for previous errors.
 *
 * This function drives the vehicle forward to reach a target distance (BLOCK_SIZE)
 * while maintaining a straight heading. It applies heading compensation and a linear
 * deceleration profile as the target distance is approached.
 *
 * @param direction The compass direction to drive towards.
 * @param heading_compensation Heading compensation in degrees (added to the initial yaw).
 * @param distance_compensation Distance compensation in cm (subtracted from BLOCK_SIZE).
 * @return drive_result_t Structure containing final heading error and other sensor readings.
 */
drive_result_t motor_forward_distance(Direction direction, float heading_compensation, float distance_compensation)
{
    drive_result_t result = {0};
    
    // Calculate compensated distance
    float compensated_distance = BLOCK_SIZE - distance_compensation;
    
    // Get current heading for logging
    float current_yaw = mpu_get_orientation().yaw;
    
    // Only log yaw-related information
    log_remote("[Drive Start] Direction: %d°, Heading Comp: %.2f°, Current Yaw: %.2f°, Distance: %.1f cm",
               direction, heading_compensation, current_yaw, compensated_distance);
    
    // Initialize drive parameters
    encoder_reset();
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long last_pid_time = start_time;
    float last_distance = 0.0f;
    unsigned long still_time = 0;
    
    // Constants for drive control
    const unsigned long STILL_THRESHOLD_MS = 250;  // 0.25 seconds threshold for stationary detection
    const float DECEL_THRESHOLD_CM = 30.0f;        // Begin deceleration when 30 cm remain
    const float STATIONARY_THRESHOLD_CM = 0.1f;    // Threshold for detecting stationary state
    const float ALIGNMENT_START_CM = 10.0f;        // Start reducing heading compensation when 20cm remain
    
    // Set initial motor direction and speed
    set_motor_direction(true);  // Set both motors to forward
    unsigned long base_duty = (unsigned long)(MOTOR_SPEED * PWM_MAX_DUTY);
    set_motor_speed(base_duty, base_duty);
    
    float current_distance = 0.0f;
    
    // Main drive loop
    while (true) {
        // Get current state
        current_distance = encoder_get_distance();
        mpu_data_t orientation = mpu_get_orientation();
        current_yaw = orientation.yaw;
        
        // Check for stationary condition after initial movement
        if (current_distance > 35.0f) {
            if (fabs(current_distance - last_distance) < STATIONARY_THRESHOLD_CM) {
                still_time += 10;
                if (still_time >= STILL_THRESHOLD_MS) {
                    log_remote("[Drive Stop] Vehicle stationary at %.2f cm", current_distance);
                    break;
                }
            } else {
                still_time = 0;
            }
        }
        last_distance = current_distance;
        
        // Check if target distance reached
        if (current_distance >= compensated_distance) {
            log_remote("[Drive Complete] Reached target distance: %.2f cm", current_distance);
            break;
        }
        
        // Calculate speed based on remaining distance
        float remaining = compensated_distance - current_distance;
        float speed = compute_decelerated_speed(MOTOR_SPEED, remaining, DECEL_THRESHOLD_CM);
        
        // Gradually reduce heading compensation as we approach target
        float current_compensation = heading_compensation;
        if (remaining < ALIGNMENT_START_CM) {
            // Linearly reduce compensation from ALIGNMENT_START_CM to 0
            current_compensation = heading_compensation * (remaining / ALIGNMENT_START_CM);
        }
        
        // Calculate target yaw with current compensation
        float target_yaw = (float)direction + current_compensation;
        target_yaw = normalize_angle(target_yaw);
        
        // Calculate heading error and correction
        float yaw_error = calculate_yaw_error(target_yaw, current_yaw);
        unsigned long current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (current_time - last_pid_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        last_pid_time = current_time;
        
        float correction = calculate_heading_correction(yaw_error, dt);
        
        // Apply speed and correction to motor duties
        float duty_ratio_a = speed + correction;
        float duty_ratio_b = speed - correction;
        
        // Ensure duty ratios stay within bounds
        duty_ratio_a = fmaxf(0.0f, fminf(duty_ratio_a, 1.0f));
        duty_ratio_b = fmaxf(0.0f, fminf(duty_ratio_b, 1.0f));
        
        long duty_a = (long)(duty_ratio_a * PWM_MAX_DUTY + 0.5f);
        long duty_b = (long)(duty_ratio_b * PWM_MAX_DUTY + 0.5f);
        
        set_motor_speed(duty_a, duty_b);
        
        // Only log yaw-related information
        log_remote("[Drive Progress] Dist: %.2f cm, Remaining: %.2f cm, Yaw: %.2f°, Target: %.2f°, Comp: %.2f°, Error: %.2f°, Speed: %.0f%%",
                   current_distance, remaining, current_yaw, target_yaw, current_compensation, yaw_error, speed * 100.0f);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Stop motors and wait for vehicle to come to rest
    motor_stop();
    wait_for_stop(STILL_THRESHOLD_MS, &result, (float)direction);
    
    // Get final state
    result.heading = mpu_get_orientation().yaw;
    result.ultrasonic = ultrasonic_get_all();
    result.distance = current_distance;
    // Only log yaw-related information
    log_remote("[Drive End] Final distance: %.2f cm, Final heading: %.2f°, Heading error: %.2f°",
               result.distance, result.heading, calculate_yaw_error((float)direction, result.heading));
    
    return result;
}

/**
 * @brief Rotate the car to a specified direction with a heading offset.
 *
 * The function rotates the car to the target base direction (e.g., north: 0°, east: 90°, etc.)
 * while applying a heading offset. The desired heading is computed as:
 *     desired_heading = normalize_angle(target_direction + heading_offset)
 * This means if you specify target_direction=0 (north) and heading_offset=-10, the car will try to
 * achieve a heading of 350° (which is equivalent to -10°).
 *
 * The turning uses its own PID controller (with TURN_PID_* constants) to command the motors in a
 * tank-turn configuration (one motor forward, the other in reverse). The function runs until the error
 * is within TURN_TOLERANCE or a timeout occurs.
 *
 * @param target_direction The base target direction in degrees (e.g., 0, 90, 180, or 270).
 * @param heading_offset The offset in degrees to add (or subtract) from the base target.
 * @return drive_result_t Structure containing the final heading error and sensor data.
 */
static float calculate_turn_correction(float error, float dt) {
    static float turn_integral = 0.0f;
    static float last_turn_error = 0.0f;
    
    if (dt < 0.001f)
        dt = 0.001f;
    
    if (fabs(error) < TURN_DEADZONE) {
        error = 0;
        turn_integral = 0;
    }
    
    turn_integral += error * dt;
    if (turn_integral > TURN_PID_I_MAX)
        turn_integral = TURN_PID_I_MAX;
    else if (turn_integral < -TURN_PID_I_MAX)
        turn_integral = -TURN_PID_I_MAX;
    
    float derivative = (error - last_turn_error) / dt;
    float output = TURN_PID_KP * error + TURN_PID_KI * turn_integral + TURN_PID_KD * derivative;
    last_turn_error = error;
    return output;
}

drive_result_t motor_rotate_to_direction(float target_direction, float heading_offset) {
    drive_result_t result;
    const float LEFT_MOTOR_COMPENSATION = 1.2f;  // Compensation factor for left motor's lower torque
    
    // Compute the desired final heading (normalized to [0,360))
    float desired_heading = normalize_angle(target_direction + heading_offset);
    log_remote("Rotating to direction: %.2f° (base: %.2f° with offset %.2f°)", desired_heading, target_direction, heading_offset);
    
    // Stop motors before starting the turn
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long current_time = start_time;
    last_pid_update = start_time;
    
    const unsigned long STABLE_TIME_REQUIRED_MS = 300;
    unsigned long within_tolerance_time = 0;
    
    while (true) {
        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - start_time > TURN_TIMEOUT_MS) {
            log_remote("Turn timed out after %lu ms", current_time - start_time);
            break;
        }
        
        float current_heading = mpu_get_orientation().yaw;
        float error = calculate_yaw_error(desired_heading, current_heading);
        
        if (fabs(error) < TURN_TOLERANCE) {
            within_tolerance_time += 10;
            if (within_tolerance_time >= STABLE_TIME_REQUIRED_MS) {
                log_remote("Desired heading reached: current heading %.2f° within tolerance %.2f°", current_heading, TURN_TOLERANCE);
                break;
            }
        } else {
            within_tolerance_time = 0;
        }
        
        float dt = (current_time - last_pid_update) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        last_pid_update = current_time;
        
        float pid_output = calculate_turn_correction(error, dt);
        float base_turn_duty = TURN_SPEED * PWM_MAX_DUTY;
        // Scale the PID correction relative to the PWM duty cycle
        float turn_command = base_turn_duty + (pid_output * PWM_MAX_DUTY / 180.0f);
        
        if (turn_command > MAX_DUTY_RATIO * PWM_MAX_DUTY)
            turn_command = MAX_DUTY_RATIO * PWM_MAX_DUTY;
        if (turn_command < MIN_DUTY_RATIO * PWM_MAX_DUTY)
            turn_command = MIN_DUTY_RATIO * PWM_MAX_DUTY;
        
        // Determine turn direction and apply left motor compensation
        int direction = (error >= 0) ? 1 : -1;
        long right_duty = (long)(-turn_command * direction);
        long left_duty = (long)(turn_command * direction * LEFT_MOTOR_COMPENSATION);  // Apply compensation to left motor
        
        set_motor_speed(right_duty, left_duty);
        
        log_remote("Turning: Current=%.2f°, Target=%.2f°, Error=%.2f°, PID=%.2f, Right=%ld, Left=%ld (compensated)", 
                    current_heading, desired_heading, error, pid_output, right_duty, left_duty);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    motor_stop();
    
    float final_heading = mpu_get_orientation().yaw;
    float final_error = calculate_yaw_error(desired_heading, final_heading);
    result.heading = final_error;
    result.ultrasonic = ultrasonic_get_all();
    
    log_remote("Rotation complete: Final heading = %.2f°, Final error = %.2f°", final_heading, final_error);
    
    return result;
}

void motor_set_direction(int motor, int direction)
{
    if (motor == MOTOR_RIGHT) {
        if (direction == MOTOR_FORWARD) {
            gpio_set_level(MOTOR_A_IN1, 1);  // Input 1 HIGH
            gpio_set_level(MOTOR_A_IN2, 0);  // Input 2 LOW
            // log_remote("Right motor set to FORWARD (IN1=1, IN2=0)");
        } else {
            gpio_set_level(MOTOR_A_IN1, 0);  // Input 1 LOW
            gpio_set_level(MOTOR_A_IN2, 1);  // Input 2 HIGH
            // log_remote("Right motor set to BACKWARD (IN1=0, IN2=1)");
        }
    } else {
        if (direction == MOTOR_FORWARD) {
            gpio_set_level(MOTOR_B_IN1, 1);  // Input 3 HIGH
            gpio_set_level(MOTOR_B_IN2, 0);  // Input 4 LOW
            // log_remote("Left motor set to FORWARD (IN1=1, IN2=0)");
        } else {
            gpio_set_level(MOTOR_B_IN1, 0);  // Input 3 LOW
            gpio_set_level(MOTOR_B_IN2, 1);  // Input 4 HIGH
            // log_remote("Left motor set to BACKWARD (IN1=0, IN2=1)");
        }
    }
}

void set_motor_turn(bool turn_right)
{
    if (turn_right) {
        // Right turn: right motor backward, left motor forward
        // log_remote("Setting RIGHT turn: Right motor backward, Left motor forward");
        motor_set_direction(MOTOR_RIGHT, MOTOR_BACKWARD);
        motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    } else {
        // Left turn: right motor forward, left motor backward
        // log_remote("Setting LEFT turn: Right motor forward, Left motor backward");
        motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
        motor_set_direction(MOTOR_LEFT, MOTOR_BACKWARD);
    }
}

void motor_turn_to_cardinal(Direction target, float heading_offset)
{
    const float LEFT_MOTOR_COMPENSATION = 1.2f;  // Same compensation as in test
    const unsigned long STABLE_TIME_REQUIRED_MS = 250; // Require heading to be stable within tolerance for 250ms
    
    unsigned long within_tolerance_time = 0;
    unsigned long last_loop_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // PID constants - adjusted for more aggressive error correction
    const float Kp = 0.35f;  // Increased from 1.0 to provide more aggressive correction
    const float Ki = 0.0f;  // Keep integral at 0 to prevent windup
    const float Kd = 0.2f;  // Increased from 0.1 to provide more damping
    
    float integral = 0.0f;
    float previous_error = 0.0f;
    
    log_remote("--- Starting motor_turn_to_cardinal --- Target: %d, Offset: %.2f ---", target, heading_offset);
    
    // Get current heading
    float current_yaw = mpu_get_orientation().yaw;
    log_remote("[Turn Start] Current heading: %.2f degrees", current_yaw);
    
    // Calculate target yaw with offset
    float target_yaw = (float)target + heading_offset;
    
    // Normalize target angle to [0, 360)
    while (target_yaw >= 360.0f) target_yaw -= 360.0f;
    while (target_yaw < 0.0f) target_yaw += 360.0f;
    log_remote("[Turn Calc] Normalized Target Yaw: %.2f degrees", target_yaw);
    
    // Calculate the shortest turn error and direction
    float error = target_yaw - current_yaw;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    
    log_remote("[Turn Calc] Initial Error: %.2f degrees (will turn %s)", 
               error, error > 0 ? "counterclockwise" : "clockwise");
    
    // Check if already at target
    if (fabs(error) < MOTOR_TOLERANCE_DEG) {
        log_remote("[Turn Start] Already within tolerance (%.2f < %.2f). No turn needed.", fabs(error), MOTOR_TOLERANCE_DEG);
        motor_stop(); // Ensure motors are stopped
        return;
    }
    
    // Record start time
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    log_remote("[Turn Loop] Starting turn loop...");
    
    // Turn until target is reached, stable, or timeout
    while (1) {
        unsigned long current_loop_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        unsigned long loop_dt = current_loop_time - last_loop_time;
        last_loop_time = current_loop_time;

        // Check timeout
        if ((current_loop_time - start_time) > MOTOR_TIMEOUT_MS) {
            log_remote("[Turn Loop] Timeout reached after %lu ms", current_loop_time - start_time);
            break;
        }
        
        // Get current heading
        current_yaw = mpu_get_orientation().yaw;
        
        // Calculate remaining angle (normalized to [-180, 180])
        error = target_yaw - current_yaw;
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;
        
        // PID calculations
        integral += error * loop_dt;
        float derivative = (error - previous_error) / loop_dt;
        float pid_output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        
        // Scale PID output to duty cycle range - adjusted to maintain higher torque for small errors
        float duty_ratio = fabs(pid_output) / 35.0f; // Reduced divisor to maintain higher torque for small errors
        duty_ratio = fmaxf(0.3f, fminf(duty_ratio, 0.65f)); // Increased minimum duty ratio to 0.3
        
        // Calculate adjusted duty cycles
        long adjusted_duty = (long)(duty_ratio * PWM_MAX_DUTY);
        long adjusted_left_duty = (long)(adjusted_duty * LEFT_MOTOR_COMPENSATION);
        
        // Set motor speeds based on current error sign
        if (error > 0) {
            set_motor_speed(adjusted_duty, -adjusted_left_duty);  // Right motor forward, left motor backward
        } else {
            set_motor_speed(-adjusted_duty, adjusted_left_duty);  // Right motor backward, left motor forward
        }
        
        log_remote("[Turn Loop] Current: %.2f, Target: %.2f, Error: %.2f, Duty Ratio: %.2f, StableTime: %lu ms", 
                   current_yaw, target_yaw, error, duty_ratio, within_tolerance_time);
        
        // Check if turn is complete and stable
        if (fabs(error) < MOTOR_TOLERANCE_DEG) {
            within_tolerance_time += loop_dt;
            if (within_tolerance_time >= STABLE_TIME_REQUIRED_MS) {
                log_remote("[Turn Loop] Turn complete - heading stable within tolerance for %lu ms", within_tolerance_time);
                break;
            }
        } else {
            within_tolerance_time = 0; // Reset stable timer if we go out of tolerance
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // Increased delay slightly for stability
    }
    
    // Stop motors
    log_remote("[Turn End] Stopping motors.");
    motor_stop();
    float final_heading = mpu_get_orientation().yaw;
    log_remote("[Turn End] Final heading: %.2f degrees (Error: %.2f)", 
               final_heading, calculate_yaw_error(target_yaw, final_heading));
    log_remote("--- motor_turn_to_cardinal finished ---");
}

void test_motor_directions(void)
{
    log_remote("Starting motor rotation test sequence with compensation");
    const float TEST_SPEED = 0.55f;
    const float LEFT_COMPENSATION = 1.2f;  // Try 20% more power for left motor
    long right_duty = (long)(TEST_SPEED * PWM_MAX_DUTY);
    long left_duty = (long)(TEST_SPEED * LEFT_COMPENSATION * PWM_MAX_DUTY);
    
    log_remote("Using compensation factor %.2f for left motor (Right duty: %ld, Left duty: %ld)", 
               LEFT_COMPENSATION, right_duty, left_duty);
    
    // Test clockwise rotation (right motor backward, left motor forward)
    log_remote("Testing CLOCKWISE rotation for 2 seconds (Right=BACKWARD, Left=FORWARD)");
    motor_set_direction(MOTOR_RIGHT, MOTOR_BACKWARD);
    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, right_duty);  // Right motor
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, left_duty);   // Left motor (compensated)
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Stop and wait
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test counterclockwise rotation (right motor forward, left motor backward)
    log_remote("Testing COUNTERCLOCKWISE rotation for 2 seconds (Right=FORWARD, Left=BACKWARD)");
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
    motor_set_direction(MOTOR_LEFT, MOTOR_BACKWARD);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, right_duty);  // Right motor
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, left_duty);   // Left motor (compensated)
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Stop
    motor_stop();
    log_remote("Motor rotation test complete");
}

void test_direction_driving(void)
{
    log_remote("\n=== Starting Direction Driving Test ===\n");
    
    // Test sequence: Test each cardinal direction
    Direction directions[] = {NORTH, EAST, SOUTH, WEST};
    const int num_directions = sizeof(directions) / sizeof(directions[0]);
    
    for (int i = 0; i < num_directions; i++) {
        Direction target = directions[i];
        log_remote("\nTest %d: Direction %d° (0=N, 90=E, 180=S, 270=W)", i + 1, target);
        
        // Get initial heading
        float initial_heading = mpu_get_orientation().yaw;
        log_remote("Initial heading: %.2f°", initial_heading);
        
        // Turn to the target direction
        log_remote("Turning to target direction...");
        motor_turn_to_cardinal(target, 0.0f);
        
        // Get heading after turn
        float post_turn_heading = mpu_get_orientation().yaw;
        log_remote("Heading after turn: %.2f° (Error: %.2f°)", 
                  post_turn_heading, calculate_yaw_error((float)target, post_turn_heading));
        
        // Wait a moment to stabilize
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Drive forward in the target direction
        log_remote("Driving forward in target direction...");
        drive_result_t result = motor_forward_distance(target, 0.0f, 0.0f);
        
        // Log the results
        log_remote("Drive complete:");
        log_remote("  Final heading: %.2f° (Error: %.2f°)", 
                  result.heading, calculate_yaw_error((float)target, result.heading));
        log_remote("  Distance traveled: %.2f cm", result.distance);
        log_remote("  Ultrasonic readings - Front: %.1f cm, Left: %.1f cm, Right: %.1f cm",
                  result.ultrasonic.front, result.ultrasonic.left, result.ultrasonic.right);
        
        // Wait between tests
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    log_remote("\n=== Direction Driving Test Complete ===\n");
}

void test_navigation(void)
{
    log_remote("\n=== Starting Navigation Test ===\n");

    motor_forward_distance(NORTH, 0.0f, 0.0f);  
    
    // Test sequence: Drive in a square pattern
    Direction directions[] = {NORTH, EAST, SOUTH, WEST};
    const int num_directions = sizeof(directions) / sizeof(directions[0]);
    
    for (int i = 0; i < num_directions; i++) {
        // First turn to the target direction
        Direction target = directions[i];
        log_remote("\nTest %d: Turning to direction %d (0=N, 90=E, 180=S, 270=W)", i + 1, target);
        
        // Get initial heading before turn
        float initial_heading = mpu_get_orientation().yaw;
        log_remote("Initial heading before turn: %.2f degrees", initial_heading);
        
        // Perform the turn with no offset
        motor_turn_to_cardinal(target, 0.0f);
        
        // Get heading after turn
        float post_turn_heading = mpu_get_orientation().yaw;
        log_remote("Heading after turn: %.2f degrees", post_turn_heading);
        
        // Wait a moment to stabilize
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Now drive forward one block
        log_remote("Driving forward one block...");
        drive_result_t result = motor_forward_distance(target, 0.0f, 0.0f);  // No compensation for first attempt
        
        // Log the results
        log_remote("Forward movement complete:");
        log_remote("  Final heading error: %.2f degrees", result.heading);
        log_remote("  Ultrasonic readings - Front: %.1f cm, Left: %.1f cm, Right: %.1f cm",
                  result.ultrasonic.front, result.ultrasonic.left, result.ultrasonic.right);
        
        // Wait between segments
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Final turn back to North to complete the square
    log_remote("\nFinal turn back to North...");
    motor_turn_to_cardinal(NORTH, 0.0f);
    
    log_remote("\n=== Navigation Test Complete ===\n");
}

void test_turning(void)
{
    log_remote("\n=== Starting Turning Test ===\n");
    
    // Test sequence: Test each cardinal direction
    Direction directions[] = {NORTH, EAST, SOUTH, WEST};
    const int num_directions = sizeof(directions) / sizeof(directions[0]);
    
    for (int i = 0; i < num_directions; i++) {
        Direction target = directions[i];
        log_remote("\nTest %d: Turning to %d° (0=N, 90=E, 180=S, 270=W)", i + 1, target);
        
        // Get initial heading
        float initial_heading = mpu_get_orientation().yaw;
        log_remote("Initial heading: %.2f°", initial_heading);
        
        // Calculate required turn
        float required_turn = (float)target - initial_heading;
        if (required_turn > 180.0f) required_turn -= 360.0f;
        if (required_turn < -180.0f) required_turn += 360.0f;
        log_remote("Required turn: %.2f° (%s)", 
                  required_turn, 
                  required_turn > 0 ? "counterclockwise" : "clockwise");
        
        // Perform the turn
        log_remote("Executing turn...");
        motor_turn_to_cardinal(target, 0.0f);
        
        // Get final heading
        float final_heading = mpu_get_orientation().yaw;
        float final_error = calculate_yaw_error((float)target, final_heading);
        
        log_remote("Turn complete:");
        log_remote("  Initial heading: %.2f°", initial_heading);
        log_remote("  Target heading: %.2f°", (float)target);
        log_remote("  Final heading: %.2f°", final_heading);
        log_remote("  Final error: %.2f°", final_error);
        
        // Wait between tests
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    log_remote("\n=== Turning Test Complete ===\n");
} 