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
#define MOTOR_A_IN1 37 // Direction pin 1
#define MOTOR_A_IN2 48 // Direction pin 2
#define MOTOR_A_EN 38  // Enable pin (PWM)

#define MOTOR_B_IN1 35 // Direction pin 1
#define MOTOR_B_IN2 36 // Direction pin 2
#define MOTOR_B_EN 45  // Enable pin (PWM)

#define PWM_CHANNEL_A LEDC_CHANNEL_0
#define PWM_CHANNEL_B LEDC_CHANNEL_1
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION 13
#define PWM_FREQ 5000
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION) - 1)

// ============================================================================
// PID Control Parameters
// ============================================================================
#define PID_KP 0.03f
#define PID_KI 0.0002f
#define PID_KD 0.00035f
#define PID_I_MAX 0.03f
#define PID_UPDATE_MS 10     
#define DRIVE_TIME_MS 5000   
#define PID_DEADZONE 0.25f   
#define MAX_CORRECTION 0.3f  
#define MIN_DUTY_RATIO 0.05f 
#define MAX_DUTY_RATIO 0.9f  

// Additional parameters for turning and calibration (not used directly in this function)
#define TURN_PID_KP 0.3f            
#define TURN_PID_KI 0.01f          
#define TURN_PID_KD 0.05f         
#define TURN_PID_I_MAX 0.5f       
#define TURN_SPEED 0.65f            
#define TURN_TOLERANCE 2.0f         
#define TURN_TIMEOUT_MS 3000        
#define MIN_TURN_TIME_MS 300        
#define CALIBRATION_SPEED 0.3f      
#define CALIBRATION_TIME_MS 2000    
#define MIN_MOTOR_FORCE 0.4f        
#define TURN_DEADZONE 1.0f          
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
        
        log_remote("Coasting - Distance: %.2f cm, Yaw: %.2f°, Distance change: %.2f cm",
                   current_distance, orientation.yaw, distance_change);
        
        if (distance_change < 0.1f) {  
            coasting_still_time += 10;
            if (coasting_still_time >= still_threshold_ms) {
                is_moving = false;
                float final_yaw_error = calculate_yaw_error(target_yaw, orientation.yaw);
                result->heading = final_yaw_error;
                result->ultrasonic = ultrasonic_get_all();
                log_remote("Car has come to a complete stop. Final distance: %.2f cm (overshoot: %.2f cm), Final yaw: %.2f° (error: %.2f°), Ultrasonic: Front=%.1f cm, Left=%.1f cm, Right=%.1f cm",
                           current_distance, current_distance - BLOCK_SIZE, orientation.yaw, final_yaw_error,
                           result->ultrasonic.front, result->ultrasonic.left, result->ultrasonic.right);
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
    log_remote("Initializing motor driver...");
    
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
    
    log_remote("Motor driver initialization complete");
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

/**
 * @brief Drive forward for a specific distance with compensation for previous errors.
 *
 * This function drives the vehicle forward to reach a target distance (BLOCK_SIZE)
 * while maintaining a straight heading. It applies heading compensation and a linear
 * deceleration profile as the target distance is approached.
 *
 * @param heading_compensation Heading compensation in degrees (added to the initial yaw).
 * @param distance_compensation Distance compensation in cm (subtracted from BLOCK_SIZE).
 * @return drive_result_t Structure containing final heading error and other sensor readings.
 */
drive_result_t motor_forward_distance(float heading_compensation, float distance_compensation)
{
    drive_result_t result;
    // Apply distance compensation
    float compensated_distance = BLOCK_SIZE - distance_compensation;
    
    // Reset encoders and record initial heading
    encoder_reset();
    float initial_yaw = mpu_get_orientation().yaw;
    // Normalize the target heading to the 0–360° range
    float target_yaw = normalize_angle(initial_yaw + heading_compensation);
    
    log_remote("Starting drive for %.1f cm (compensated: %.1f cm) at max speed: %.0f%%, target yaw: %.2f° (compensated by %.2f°)",
               BLOCK_SIZE, compensated_distance, MOTOR_SPEED * 100.0f, target_yaw, heading_compensation);
    
    // Set up motors for forward motion
    set_motor_direction(true);
    unsigned long base_duty = (unsigned long)(MOTOR_SPEED * PWM_MAX_DUTY);
    set_motor_speed(base_duty, base_duty);
    
    unsigned long start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_pid_update = start_time;
    
    float last_distance = encoder_get_distance();
    unsigned long still_time = 0;
    const unsigned long STILL_THRESHOLD_MS = 250; // 0.25 seconds threshold
    const float DECEL_THRESHOLD_CM = 30.0f;         // Begin deceleration when 30 cm remain
    
    // Main drive loop
    while (true) {
        float current_distance = encoder_get_distance();
        
        // After 35 cm of travel, check if the car has become stationary.
        if (current_distance > 35.0f) {
            if (is_stationary(last_distance, current_distance, 0.1f)) {
                still_time += 10;
                if (still_time >= STILL_THRESHOLD_MS) {
                    log_remote("Car stopped moving before reaching target distance. Current distance: %.2f cm", current_distance);
                    break;
                }
            }
            else {
                still_time = 0;
            }
        }
        last_distance = current_distance;
        
        // If we've reached or surpassed the compensated distance, exit the loop.
        if (current_distance >= compensated_distance) {
            break;
        }
        
        // Determine the decelerated speed.
        float remaining = compensated_distance - current_distance;
        float speed = compute_decelerated_speed(MOTOR_SPEED, remaining, DECEL_THRESHOLD_CM);
        
        // Get current heading and compute the heading error for PID correction.
        mpu_data_t orientation = mpu_get_orientation();
        float yaw_error = calculate_yaw_error(target_yaw, orientation.yaw);
        
        // Update the PID controller based on the elapsed time.
        unsigned long new_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (new_time - last_pid_update) / 1000.0f;
        if (dt < 0.001f)
            dt = 0.001f;
        last_pid_update = new_time;
        float correction = calculate_heading_correction(yaw_error, dt);
        
        // Adjust motor speeds based on PID correction.
        float duty_ratio_a = speed + correction;
        float duty_ratio_b = speed - correction;
        long duty_a = (long)(duty_ratio_a * PWM_MAX_DUTY + 0.5f);
        long duty_b = (long)(duty_ratio_b * PWM_MAX_DUTY + 0.5f);
        set_motor_speed(duty_a, duty_b);
        
        log_remote("Distance: %.2f cm, Remaining: %.2f cm, Yaw: %.2f°, Error: %.2f°, Correction: %.2f, Speed: %.0f%%, Duty A: %ld, Duty B: %ld",
                   current_distance, remaining, orientation.yaw, yaw_error, correction, speed * 100.0f, duty_a, duty_b);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    log_remote("Target distance reached (%.2f cm). Stopping motors.", encoder_get_distance());
    motor_stop();
    
    // Allow the vehicle to coast to a stop and record the final status.
    wait_for_stop(STILL_THRESHOLD_MS, &result, target_yaw);
    
    return result;
}
