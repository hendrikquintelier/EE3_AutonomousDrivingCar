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
#define PID_KP 0.005f         // Further reduced for gentler response
#define PID_KI 0.0001f       // Reduced integral action
#define PID_KD 0.0015f      // Increased derivative gain for better damping
#define PID_I_MAX 0.03f      // Reduced integral windup limit
#define PID_UPDATE_MS 10     // Reduced update interval for more responsive control
#define DRIVE_TIME_MS 5000   // Total drive time (10 seconds)
#define PID_DEADZONE 0.25f   // Reduced deadzone to 0.5 degrees for more precise control

// ============================================================================
// Turn Control Parameters
// ============================================================================
#define TURN_SPEED 0.45f     // Speed for turning
#define TURN_TOLERANCE 2.0f  // Tolerance for turn completion
#define MIN_TURN_TIME_MS 300 // Minimum time for a turn

// Additional parameters for turning and calibration (not used directly in this function)
#define TURN_PID_KP 0.05f            
#define TURN_PID_KI 0.001f          
#define TURN_PID_KD 0.005f         
#define TURN_PID_I_MAX 0.5f       
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
    
    // Clip output to maximum of 0.25
    if (output > 0.25f)
        output = 0.25f;
    else if (output < -0.25f)
        output = -0.25f;
        
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
Direction current_direction = NORTH;
/* ───────────────────────────────────────────────────────────────────────────
 *  Adaptive duty profile helpers
 * ──────────────────────────────────────────────────────────────────────────*/
static float duty_profile(float duty_max,
                          float remaining,
                          float decel_start,
                          float brake_start)
{
    /* Cruise */
    if (remaining > decel_start)
        return duty_max;

    /* Linear taper  →  min 40 % duty at brake-start                        */
    if (remaining > brake_start) {
        float s = (remaining - brake_start) / (decel_start - brake_start); // 1→0
        return duty_max * (0.40f + 0.60f * s);
    }

    /* Brake zone  (reverse-torque grows to −25 %)                          */
    float b = (brake_start > 1.0f) ? (remaining / brake_start) : 0.0f;      // 1→0
    return -duty_max * 0.25f * b;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Refactored forward drive
 * ──────────────────────────────────────────────────────────────────────────*/
drive_result_t motor_forward_distance(float heading_comp, float dist_comp)
{
    /* Tunables ----------------------------------------------------------- */
    const float RAMP_START_DUTY   = 0.30f;  /* soft-start 30 %             */
    const float RAMP_STEP         = 0.02f;  /* +2 % each loop              */
    const float CRUISE_DUTY_MAX   = MOTOR_SPEED;
    const float RAMP_SPEED_CMPS   = 10.0f;  /* finish ramp when >10 cm/s   */
    const unsigned LOOP_MS        = 50;     /* control period              */
    const float STATIC_THRESHOLD  = 0.1f;   /* cm/s - considered static    */
    const unsigned STATIC_TIME_MS = 500;    /* time to be static before exit */
    /* -------------------------------------------------------------------- */

    /* 1 )  Figure out how far to drive using the front ultrasonic sensor  */
    ultrasonic_readings_t u = ultrasonic_get_all();
    float front = (u.front < 1.0f || u.front > 400.0f) ? 0.0f : u.front;

    float raw_off = fmodf(front, BLOCK_SIZE) - 10.0f;   /* −10 … +30 cm      */
    if (raw_off >  8.0f) raw_off =  8.0f;
    if (raw_off < -8.0f) raw_off = -8.0f;

    float target_dist = BLOCK_SIZE + raw_off - dist_comp;        /* cm      */

    log_remote("[FWD] front=%.1f cm  offset=%.1f → target=%.1f cm",
               front, raw_off, target_dist);

    /* 2 )  Setup -------------------------------------------------------- */
    drive_result_t result;
    encoder_reset();

    float base_yaw   = current_direction;
    float target_yaw = normalize_angle(base_yaw + heading_comp);

    float duty_ratio  = RAMP_START_DUTY;
    float last_dist   = 0.0f;
    unsigned long last_ms = xTaskGetTickCount()*portTICK_PERIOD_MS;
    last_pid_update      = last_ms;
    unsigned long static_start_ms = 0;
    bool is_static = false;

    set_motor_direction(true);

    /* 3 )  Main loop ----------------------------------------------------- */
    while (true) {
        /* time & odometry */
        unsigned long now_ms = xTaskGetTickCount()*portTICK_PERIOD_MS;
        float dt = (now_ms - last_ms)/1000.0f; if (dt < 0.001f) dt = 0.001f;

        float dist = encoder_get_distance();
        float speed = (dist - last_dist)/dt;                        /* cm/s */
        float remaining = target_dist - dist; if (remaining < 0) remaining = 0;

        /* Check if car is static */
        if (fabs(speed) < STATIC_THRESHOLD) {
            if (!is_static) {
                static_start_ms = now_ms;
                is_static = true;
            } else if ((now_ms - static_start_ms) >= STATIC_TIME_MS) {
                /* Only exit if we've traveled at least 70% of the target distance */
                if (dist >= 0.7f * target_dist) {
                    log_remote("[FWD] Car is static for %lu ms and traveled %.1f/%.1f cm (%.0f%%), exiting", 
                              STATIC_TIME_MS, dist, target_dist, (dist/target_dist)*100.0f);
                    break;
                } else {
                    log_remote("[FWD] Car is static but only traveled %.1f/%.1f cm (%.0f%%), continuing", 
                              dist, target_dist, (dist/target_dist)*100.0f);
                    is_static = false;  // Reset static state to try again
                }
            }
        } else {
            is_static = false;
        }

        /* dynamic thresholds */
        const float DECEL_START = fmaxf(0.40f*target_dist, 12.0f);  /* ≥12 cm */
        const float BRAKE_START = 1.0f;                             /* fixed */

        /* a) ramp-up until >10 cm/s */
        if (speed < RAMP_SPEED_CMPS && duty_ratio < CRUISE_DUTY_MAX) {
            duty_ratio += RAMP_STEP;
            if (duty_ratio > CRUISE_DUTY_MAX) duty_ratio = CRUISE_DUTY_MAX;
        }

        /* b) adaptive profile (overrides ramp once cruising) */
        duty_ratio = duty_profile(duty_ratio, remaining,
                                  DECEL_START, BRAKE_START);

        /* heading PID */
        mpu_data_t ori = mpu_get_orientation();
        float yaw_err  = calculate_yaw_error(target_yaw, ori.yaw);
        float dt_pid   = (now_ms - last_pid_update)/1000.0f;
        if (dt_pid < 0.001f) dt_pid = 0.001f;
        last_pid_update = now_ms;

        float corr = calculate_heading_correction(yaw_err, dt_pid);

        long duty_a = (long)((duty_ratio + corr)*PWM_MAX_DUTY + 0.5f);
        long duty_b = (long)((duty_ratio - corr)*PWM_MAX_DUTY + 0.5f);
        set_motor_speed(duty_a, duty_b);

        log_remote("[FWD] d=%.2f/%.1f  v=%.1f cm/s  rem=%.1f  duty=%.0f%%  yawErr=%.1f°",
                   dist, target_dist, speed, remaining,
                   duty_ratio*100.0f, yaw_err);

        /* exit when we've covered the distance */
        if (dist >= target_dist) break;

        last_dist = dist;
        last_ms   = now_ms;
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }

    /* stop, coast & gather result */
    motor_stop();
    wait_for_stop(250, &result, target_yaw);
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
    current_direction = target_direction;
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
            log_remote("Right motor set to FORWARD (IN1=1, IN2=0)");
        } else {
            gpio_set_level(MOTOR_A_IN1, 0);  // Input 1 LOW
            gpio_set_level(MOTOR_A_IN2, 1);  // Input 2 HIGH
            log_remote("Right motor set to BACKWARD (IN1=0, IN2=1)");
        }
    } else {
        if (direction == MOTOR_FORWARD) {
            gpio_set_level(MOTOR_B_IN1, 1);  // Input 3 HIGH
            gpio_set_level(MOTOR_B_IN2, 0);  // Input 4 LOW
            log_remote("Left motor set to FORWARD (IN1=1, IN2=0)");
        } else {
            gpio_set_level(MOTOR_B_IN1, 0);  // Input 3 LOW
            gpio_set_level(MOTOR_B_IN2, 1);  // Input 4 HIGH
            log_remote("Left motor set to BACKWARD (IN1=0, IN2=1)");
        }
    }
}

void set_motor_turn(bool turn_right)
{
    if (turn_right) {
        // Right turn: right motor backward, left motor forward
        log_remote("Setting RIGHT turn: Right motor backward, Left motor forward");
        motor_set_direction(MOTOR_RIGHT, MOTOR_BACKWARD);
        motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    } else {
        // Left turn: right motor forward, left motor backward
        log_remote("Setting LEFT turn: Right motor forward, Left motor backward");
        motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
        motor_set_direction(MOTOR_LEFT, MOTOR_BACKWARD);
    }
}

void motor_turn_to_cardinal(Direction target, float heading_offset)
{
    current_direction = target;
    const float LEFT_MOTOR_COMPENSATION = 1.2f;  // Same compensation as in test
    const unsigned long STABLE_TIME_REQUIRED_MS = 250; // Require heading to be stable within tolerance for 250ms
    
    unsigned long within_tolerance_time = 0;
    unsigned long last_loop_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // PID constants - adjusted for more aggressive error correction
    const float Kp = 0.2f;  // Increased from 1.0 to provide more aggressive correction
    const float Ki = 0.0f;  // Keep integral at 0 to prevent windup
    const float Kd = 0.05f;  // Increased from 0.1 to provide more damping
    
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



/******************************************************************************
 *  motor_turn_to_cardinal_slow
 *
 *  Slow, accurate 90 ° (cardinal) turn with soft-start and soft-stop.
 *  ──────────────────────────────────────────────────────────────────────────
 *  1.  *Ramp-up* : increase PWM in 2 % steps every 15 ms until angular speed
 *      exceeds 10 °/s (or the max ramp duty is reached).
 *  2.  *Coarse zone* (|err| ≥ 60 °) : keep last ramp duty.
 *  3.  *Proportional zone* (15 ° < |err| < 60 °) : linearly drop duty
 *      0.60 → 0.0 as error shrinks 60→15 °.
 *  4.  *Brake zone* (|err| ≤ 15 °) : reverse-torque rises 0 → 0.25 duty.
 *  5.  *Overshoot* : if sign(error) flips, fire a 0.30 duty brake pulse
 *      opposite to motion and end the routine.
 ******************************************************************************/
void motor_turn_to_cardinal_slow(Direction target, float heading_offset)
{
    /* ───────────── tweakables ──────────────────────────────────────────── */
    const float  LEFT_FACTOR              = 1.0f;  /* compensate weaker left */
    const float  RAMP_START_DUTY          = 0.15f;  /* 15 % duty               */
    const float  RAMP_MAX_DUTY            = 0.65f;  /* 60 % duty upper bound   */
    const float  RAMP_STEP                = 0.025f;  /* +2 % each loop          */
    const float  MIN_ANG_VEL_DPS          = 20.0f;  /* stop ramp when >10 °/s  */

    const float  COARSE_TO_PROP_DEG       = 50.0f;
    const float  BRAKE_ZONE_BEGIN_DEG     = 12.0f;
    const float  MAX_PROP_DUTY            = 0.60f;
    const float  MAX_BRAKE_DUTY           = 0.25f;
    const float  OVERSHOOT_BRAKE_DUTY     = 0.375f;  // Changed from 0.20f to 0.30f to match comment

    const unsigned STABLE_TIME_REQUIRED_MS= 300;
    const unsigned LOOP_PERIOD_MS         = 15;
    /* ───────────────────────────────────────────────────────────────────── */

    /* Keep global state aligned with the new facing.                       */
    current_direction = target;

    /* ----- derive final heading ----- */
    float desired_heading = normalize_angle((float)target + heading_offset);

    /* ----- get initial orientation & error ----- */
    float heading      = mpu_get_orientation().yaw;
    float error        = calculate_yaw_error(desired_heading, heading);
    int   dir_sign     = (error >= 0.0f) ? +1 : -1;          /* +1 CCW, -1 CW */

    log_remote("[TURN-SLOW] start heading %.2f°, want %.2f° (err %.2f°)",
               heading, desired_heading, error);

    /******************************  RAMP-UP  ********************************/
    float duty_ratio   = RAMP_START_DUTY;
    float last_yaw     = heading;
    unsigned long last_tick_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (duty_ratio <= RAMP_MAX_DUTY)
    {
        /* apply current duty */
        long duty  = (long)(duty_ratio * PWM_MAX_DUTY);
        long right =  dir_sign * duty;                         /* sign fixed  */
        long left  = -dir_sign * duty * LEFT_FACTOR;
        set_motor_speed(right, left);

        /* wait one control period */
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));

        /* measure angular velocity */
        unsigned long now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt   = (now_ms - last_tick_ms) / 1000.0f;        /* s           */
        if (dt < 0.001f) dt = 0.001f;

        float yaw   = mpu_get_orientation().yaw;
        float dYaw  = calculate_yaw_error(yaw, last_yaw);      /* signed diff */
        float yaw_rate = fabsf(dYaw) / dt;                     /* °/s         */

        log_remote("[TURN-SLOW][RAMP] duty %.0f %%  ω=%.1f °/s",
                   duty_ratio * 100.0f, yaw_rate);

        /* stop ramping once turning briskly enough */
        if (yaw_rate > MIN_ANG_VEL_DPS) break;

        /* otherwise bump duty & iterate */
        duty_ratio += RAMP_STEP;
        if (duty_ratio > RAMP_MAX_DUTY) duty_ratio = RAMP_MAX_DUTY;

        last_yaw     = yaw;
        last_tick_ms = now_ms;
    }
    const float cruise_duty = duty_ratio;   /* remember last ramp duty */

    /************************  MAIN CONTROL LOOP  ***************************/
    unsigned long stable_ms   = 0;
    float previous_error      = error;
    float last_heading       = heading;

    while (true)
    {
        unsigned long start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        heading = mpu_get_orientation().yaw;
        error   = calculate_yaw_error(desired_heading, heading);
        float abs_err = fabsf(error);

        /* -------- check for static condition with small error -------- */
        float heading_change = fabsf(calculate_yaw_error(heading, last_heading));
        if (heading_change < 0.1f && abs_err < 10.0f) {  // Static and error < 10 degrees
            log_remote("[TURN-SLOW] Static with small error (%.2f°), exiting turn", abs_err);
            break;
        }
        last_heading = heading;

        /* -------- overshoot brake -------- */
        if ((previous_error > 0 && error < 0) ||
            (previous_error < 0 && error > 0))
        {
            long brake = (long)(OVERSHOOT_BRAKE_DUTY * PWM_MAX_DUTY);
            long right =  dir_sign * brake;                     /* invert OK  */
            long left  = -dir_sign * brake * LEFT_FACTOR;
            set_motor_speed(right, left);
            log_remote("[TURN-SLOW] overshoot! brake %.0f %% for 200 ms",
                       OVERSHOOT_BRAKE_DUTY * 100.0f);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }

        /* -------- choose duty based on error magnitude -------- */
        float cmd_ratio;      /* signed – positive = CCW torque */
        if (abs_err >= COARSE_TO_PROP_DEG)
        {
            cmd_ratio = cruise_duty * dir_sign;                 /* coarse     */
        }
        else if (abs_err > BRAKE_ZONE_BEGIN_DEG)
        {
            /* linear fall 60 °→15 ° maps 0.60→0.0 duty */
            float scale = (abs_err - BRAKE_ZONE_BEGIN_DEG) /
                          (COARSE_TO_PROP_DEG - BRAKE_ZONE_BEGIN_DEG);
            cmd_ratio = (MAX_PROP_DUTY * scale) * dir_sign;
        }
        else /* braking region */
        {
            /* linear rise 15 °→0 ° maps 0.0→0.25 reverse-torque */
            float scale = (BRAKE_ZONE_BEGIN_DEG - abs_err) / BRAKE_ZONE_BEGIN_DEG;
            cmd_ratio = -(MAX_BRAKE_DUTY * scale) * dir_sign;
        }

        /* -------- apply PWM to motors -------- */
        long duty  = (long)(fabsf(cmd_ratio) * PWM_MAX_DUTY);
        long right =  (cmd_ratio >= 0 ? +1 : -1) * duty;
        long left  = -(cmd_ratio >= 0 ? +1 : -1) * duty * LEFT_FACTOR;
        set_motor_speed(right, left);

        log_remote("[TURN-SLOW] err %.2f°, cmd %.0f %% (right=%ld left=%ld)",
                   error, fabsf(cmd_ratio) * 100.0f, right, left);

        /* -------- stability / exit test -------- */
        if (abs_err < MOTOR_TOLERANCE_DEG)
        {
            stable_ms += LOOP_PERIOD_MS;
            if (stable_ms >= STABLE_TIME_REQUIRED_MS) break;
        }
        else stable_ms = 0;

        previous_error = error;

        /* wait remainder of control period */
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }

    motor_stop();
    float final_heading = mpu_get_orientation().yaw;
    float final_error   = calculate_yaw_error(desired_heading, final_heading);
    log_remote("[TURN-SLOW] done – final heading %.2f° (err %.2f°)",
               final_heading, final_error);
}

/**
 *  Slow, smooth forward drive for one "block".
 *  ──────────────────────────────────────────────────────────────────────────
 *  1.  Duty starts at 15 % and rises by 2 % every 50 ms while
 *      measured speed < 10 cm·s-¹  (or until the max cruise duty is hit).
 *  2.  Once cruising, the duty is capped by the linear-deceleration profile
 *      (same as in the fast routine) so the bot eases into the stop point.
 *  3.  Heading is kept with the same PID used in `motor_forward_distance()`.
 *  4.  Instantaneous speed is logged (cm · s-¹).
 */
drive_result_t motor_forward_distance_slow(float heading_comp, float dist_comp)
{
    /* ───── tunables ───── */
    const float RAMP_START_DUTY   = 0.3f;   /* 15 %                       */
    const float RAMP_STEP         = 0.02f;   /* +2 % every loop            */
    const float CRUISE_DUTY_MAX   = MOTOR_SPEED;      /* from config       */
    const float START_SPEED_CMPS  = 10.0f;   /* stop ramp when faster      */
    const unsigned LOOP_MS        = 50;      /* slower loop is OK          */
    /* ───────────────────── */

    drive_result_t result;
    float target_dist = BLOCK_SIZE - dist_comp;                 /* cm       */

    /* encoder / IMU reset & headings */
    encoder_reset();
    float base_yaw   = current_direction;                       /* 0/90/…  */
    float target_yaw = normalize_angle(base_yaw + heading_comp);

    log_remote("[FWD-SLOW] drive %.1f cm  (target yaw %.2f°, comp %.2f°)",
               target_dist, target_yaw, heading_comp);

    /* soft-start variables */
    float duty_ratio      = RAMP_START_DUTY;
    float last_enc_dist   = 0.0f;
    unsigned long last_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* heading-PID vars (reuse gains / globals from fast version) */
    last_pid_update = last_ms;

    /* set forward direction */
    set_motor_direction(true);

    /* ───────────── main loop ───────────── */
    while (true)
    {
        /* time & distance deltas */
        unsigned long now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt      = (now_ms - last_ms) / 1000.0f;           /* s       */
        if (dt < 0.001f) dt = 0.001f;

        float enc_dist = encoder_get_distance();                /* cm      */
        float d_dist   = enc_dist - last_enc_dist;              /* cm      */
        float speed    = d_dist / dt;                           /* cm·s-¹  */

        /* 1 )  RAMP-UP — keep adding duty until speed threshold reached   */
        if (speed < START_SPEED_CMPS && duty_ratio < CRUISE_DUTY_MAX)
        {
            duty_ratio += RAMP_STEP;
            if (duty_ratio > CRUISE_DUTY_MAX) duty_ratio = CRUISE_DUTY_MAX;
        }

        /* 2 )  apply deceleration profile as we approach the goal         */
        float remaining = target_dist - enc_dist;
        if (remaining < 0) remaining = 0;
        float cruise_cap = compute_decelerated_speed(CRUISE_DUTY_MAX,
                                                     remaining, 30.0f);
        if (duty_ratio > cruise_cap) duty_ratio = cruise_cap;

        /* 3 )  heading PID correction                                     */
        mpu_data_t ori = mpu_get_orientation();
        float yaw_err  = calculate_yaw_error(target_yaw, ori.yaw);

        float dt_pid   = (now_ms - last_pid_update) / 1000.0f;
        if (dt_pid < 0.001f) dt_pid = 0.001f;
        last_pid_update = now_ms;

        float corr     = calculate_heading_correction(yaw_err, dt_pid);

        /* 4 )  send PWM                                                   */
        long duty_base = (long)(duty_ratio * PWM_MAX_DUTY + 0.5f);
        long duty_a    = (long)((duty_ratio + corr) * PWM_MAX_DUTY + 0.5f);
        long duty_b    = (long)((duty_ratio - corr) * PWM_MAX_DUTY + 0.5f);
        set_motor_speed(duty_a, duty_b);

        /* 5 )  logs                                                       */
        log_remote("[FWD-SLOW] d=%.2f/%.1f cm  v=%.1f cm·s-¹  yaw=%.1f° "
                   "err=%.1f°  duty=%.0f %%",
                   enc_dist, target_dist, speed,
                   ori.yaw, yaw_err, duty_ratio*100.0f);

        /* 6 )  termination checks                                         */
        if (enc_dist >= target_dist) break;

        last_enc_dist = enc_dist;
        last_ms       = now_ms;
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }

    /* stop, coast & gather result */
    motor_stop();
    wait_for_stop(250, &result, target_yaw);
    return result;
}

void test_navigation(void)
{
    log_remote("\n=== Starting Square Drive Test ===\n");
    
    // Define the sequence of directions for the square pattern
    Direction directions[] = {EAST, SOUTH, WEST, NORTH};
    const int num_directions = sizeof(directions) / sizeof(directions[0]);
    
    while(1) {
        for (int i = 0; i < num_directions; i++) {
            Direction target_direction = directions[i];
            
            // Turn to the target direction
            log_remote("\nTurning to direction %d (0=N, 90=E, 180=S, 270=W)", target_direction);
            float initial_heading = mpu_get_orientation().yaw;
            log_remote("Initial heading before turn: %.2f degrees", initial_heading);
            motor_turn_to_cardinal_slow(target_direction, 0.0f);
            float post_turn_heading = mpu_get_orientation().yaw;
            log_remote("Heading after turn: %.2f degrees", post_turn_heading);
            
            // Wait to stabilize
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // Drive forward one block
            log_remote("\nDriving forward one block");
            drive_result_t result = motor_forward_distance(0.0f, 0.0f);
            log_remote("Forward movement complete:");
            log_remote("  Final heading error: %.2f degrees", result.heading);
            log_remote("  Ultrasonic readings - Front: %.1f cm, Left: %.1f cm, Right: %.1f cm",
                       result.ultrasonic.front, result.ultrasonic.left, result.ultrasonic.right);
            
            // Wait to stabilize
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        log_remote("\n=== Square pattern completed, starting next iteration ===\n");
    }
}