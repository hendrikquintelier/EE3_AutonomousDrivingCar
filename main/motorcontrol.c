#include "exploration_algorithm/direction.h" // Provides the Direction enum: NORTH, EAST, SOUTH, WEST, etc.
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
#include "globals.h"

#define BLOCK_SIZE 40.0f

// ============================================================================
// Hardware Configuration
// ============================================================================
// Motor A pins (Right motor)
#define MOTOR_A_IN1 37 // Input 1
#define MOTOR_A_IN2 48 // Input 2
#define MOTOR_A_EN 38  // Enable 1,2

// Motor B pins (Left motor)
#define MOTOR_B_IN1 35 // Input 3
#define MOTOR_B_IN2 36 // Input 4
#define MOTOR_B_EN 45  // Enable 3,4

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
#define PID_KP 0.005f      // Further reduced for gentler response
#define PID_KI 0.0001f     // Reduced integral action
#define PID_KD 0.0015f     // Increased derivative gain for better damping
#define PID_I_MAX 0.03f    // Reduced integral windup limit
#define PID_UPDATE_MS 10   // Reduced update interval for more responsive control
#define DRIVE_TIME_MS 5000 // Total drive time (10 seconds)
#define PID_DEADZONE 0.25f // Reduced deadzone to 0.5 degrees for more precise control

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
    .is_calibrated = false};

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

/*──────────────────────────────────────────────────────────────────────────*/
/* Helper – signed, *CW-positive* heading error                            */
/*  0° = North, 90° = East, values grow clockwise.                         */
/*  Return value:                                                          */
/*     > 0  ⇒  shortest way to target is CLOCK-WISE  (right turn)          */
/*     < 0  ⇒  shortest way to target is COUNTER-CLOCK-WISE (left turn)    */
/*──────────────────────────────────────────────────────────────────────────*/
static float calculate_yaw_error(float target_yaw, float current_yaw)
{
    target_yaw = normalize_angle(target_yaw);
    current_yaw = normalize_angle(current_yaw);

    float error = target_yaw - current_yaw;
    if (error > 180.0f)
        error -= 360.0f;
    if (error < -180.0f)
        error += 360.0f;
    return error; /*  + = CW  ,  – = CCW  */
}

// Compute the deceleration-adjusted speed based on remaining distance.
static float compute_decelerated_speed(float motor_speed, float remaining, float deceleration_threshold_cm)
{
    if (remaining < deceleration_threshold_cm)
    {
        float decel_factor = remaining / deceleration_threshold_cm;
        return motor_speed * (0.475f + 0.525f * decel_factor);
    }
    return motor_speed;
}

// Check if the vehicle is effectively stationary based on encoder difference.
static bool is_stationary(float last_distance, float current_distance, float threshold)
{
    return (fabs(current_distance - last_distance) < threshold);
}

// Wait for the vehicle to come to a complete stop and log coasting status.
static void wait_for_stop(unsigned long still_threshold_ms, drive_result_t *result, float target_yaw)
{
    float coasting_last_distance = encoder_get_distance();
    unsigned long coasting_still_time = 0;
    bool is_moving = true;

    while (is_moving)
    {
        float current_distance = encoder_get_distance();
        float distance_change = fabs(current_distance - coasting_last_distance);
        mpu_data_t orientation = mpu_get_orientation();

        // log_remote("Coasting - Distance: %.2f cm, Yaw: %.2f°, Distance change: %.2f cm",
        //            current_distance, orientation.yaw, distance_change);

        if (distance_change < 0.1f)
        {
            coasting_still_time += 10;
            if (coasting_still_time >= still_threshold_ms)
            {
                is_moving = false;
                float final_yaw_error = calculate_yaw_error(target_yaw, orientation.yaw);
                result->heading = final_yaw_error;
                result->ultrasonic = ultrasonic_get_all();
                log_remote("Car has come to a complete stop. Final distance: %.2f cm (overshoot: %.2f cm), Final yaw: %.2f° (error: %.2f°), Ultrasonic: Front=%.1f cm, Left=%.1f cm, Right=%.1f cm",
                           current_distance, current_distance - BLOCK_SIZE, orientation.yaw, final_yaw_error,
                           result->ultrasonic.front, result->ultrasonic.left, result->ultrasonic.right);
            }
        }
        else
        {
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

    // log_remote("Motor speeds set - Left: %ld (%.1f%%, %s), Right: %ld (%.1f%%, %s)",
    //            duty_a, (float)duty_a / PWM_MAX_DUTY * 100.0f, motor_a_forward ? "FORWARD" : "BACKWARD",
    //            duty_b, (float)duty_b / PWM_MAX_DUTY * 100.0f, motor_b_forward ? "FORWARD" : "BACKWARD");
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
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&dir_conf);

    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

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
    if (remaining > brake_start)
    {
        float s = (remaining - brake_start) / (decel_start - brake_start); // 1→0
        return duty_max * (0.40f + 0.60f * s);
    }

    /* Brake zone  (reverse-torque grows to −25 %)                          */
    float b = (brake_start > 1.0f) ? (remaining / brake_start) : 0.0f; // 1→0
    return -duty_max * 0.25f * b;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Refactored forward drive
 * ──────────────────────────────────────────────────────────────────────────*/
drive_result_t motor_forward()
{
    float heading_comp = 0.0f;
    float dist_comp = 0.0f;
    /* Tunables ----------------------------------------------------------- */
    const float RAMP_START_DUTY = 0.30f; /* soft-start 30 %             */
    const float RAMP_STEP = 0.02f;       /* +2 % each loop              */
    const float CRUISE_DUTY_MAX = MOTOR_SPEED;
    const float RAMP_SPEED_CMPS = 10.0f; /* finish ramp when >10 cm/s   */
    const unsigned LOOP_MS = 50;         /* control period              */
    const float STATIC_THRESHOLD = 0.1f; /* cm/s - considered static    */
    const unsigned STATIC_TIME_MS = 500; /* time to be static before exit */
    /* -------------------------------------------------------------------- */

    /* 1 )  Figure out how far to drive using the front ultrasonic sensor  */
    ultrasonic_readings_t u = ultrasonic_get_all();
    float front = (u.front < 1.0f || u.front > 400.0f) ? 0.0f : u.front;

    const float MAX_DISTANCE_OFFSET = 10.0f;
    float raw_off = fmodf(front, BLOCK_SIZE) - 10.0f; /* −10 … +30 cm      */
    if (raw_off > MAX_DISTANCE_OFFSET)
        raw_off = MAX_DISTANCE_OFFSET;
    if (raw_off < -MAX_DISTANCE_OFFSET)
        raw_off = -MAX_DISTANCE_OFFSET;

    float target_dist = BLOCK_SIZE + raw_off - dist_comp;

    float current_yaw = mpu_get_orientation().yaw;

    float perpendicular_distance_right = fmod(u.right * cos(current_yaw - current_direction), 40.0f);
    float perpendicular_distance_left = fmod(u.left * cos(current_yaw - current_direction), 40.0f);

    if (perpendicular_distance_right < 10.0f && perpendicular_distance_left > 15.0f)
    {
        heading_comp = -5.0f;
        target_dist = target_dist / cos(heading_comp * M_PI / 180.0f);
    }
    else if (perpendicular_distance_right > 15.0f && perpendicular_distance_left < 10.0f)
    {
        heading_comp = 5.0f;
        target_dist = target_dist / cos(heading_comp * M_PI / 180.0f);
    }

    /* 2 )  Setup -------------------------------------------------------- */
    drive_result_t result;
    encoder_reset();

    // Convert Direction enum to degrees (NORTH=0°, EAST=90°, SOUTH=180°, WEST=270°)
    float base_yaw = (float)current_direction * 90.0f;
    float target_yaw = normalize_angle(base_yaw + heading_comp);

    float duty_ratio = RAMP_START_DUTY;
    float last_dist = 0.0f;
    unsigned long last_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_pid_update = last_ms;
    unsigned long static_start_ms = 0;
    bool is_static = false;

    set_motor_direction(true);

    /* 3 )  Main loop ----------------------------------------------------- */
    while (true)
    {
        /* time & odometry */
        unsigned long now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now_ms - last_ms) / 1000.0f;
        if (dt < 0.001f)
            dt = 0.001f;

        float dist = encoder_get_distance();
        float speed = (dist - last_dist) / dt; /* cm/s */
        float remaining = target_dist - dist;
        if (remaining < 0)
            remaining = 0;

        /* Check if car is static */
        if (fabs(speed) < STATIC_THRESHOLD)
        {
            if (!is_static)
            {
                static_start_ms = now_ms;
                is_static = true;
            }
            else if ((now_ms - static_start_ms) >= STATIC_TIME_MS)
            {
                /* Only exit if we've traveled at least 70% of the target distance */
                if (dist >= 0.7f * target_dist)
                {
                    // log_remote("[FWD] Car is static for %lu ms and traveled %.1f/%.1f cm (%.0f%%), exiting",
                    //           STATIC_TIME_MS, dist, target_dist, (dist/target_dist)*100.0f);
                    break;
                }
                else
                {
                    // log_remote("[FWD] Car is static but only traveled %.1f/%.1f cm (%.0f%%), continuing",
                    //           dist, target_dist, (dist/target_dist)*100.0f);
                    is_static = false; // Reset static state to try again
                }
            }
        }
        else
        {
            is_static = false;
        }

        /* dynamic thresholds */
        const float DECEL_START = fmaxf(0.40f * target_dist, 12.0f); /* ≥12 cm */
        const float BRAKE_START = 1.0f;                              /* fixed */

        /* a) ramp-up until >10 cm/s */
        if (speed < RAMP_SPEED_CMPS && duty_ratio < CRUISE_DUTY_MAX)
        {
            duty_ratio += RAMP_STEP;
            if (duty_ratio > CRUISE_DUTY_MAX)
                duty_ratio = CRUISE_DUTY_MAX;
        }

        /* b) adaptive profile (overrides ramp once cruising) */
        duty_ratio = duty_profile(duty_ratio, remaining,
                                  DECEL_START, BRAKE_START);

        /* heading PID */
        mpu_data_t ori = mpu_get_orientation();
        float yaw_err = calculate_yaw_error(target_yaw, ori.yaw);
        float dt_pid = (now_ms - last_pid_update) / 1000.0f;
        if (dt_pid < 0.001f)
            dt_pid = 0.001f;
        last_pid_update = now_ms;

        float corr = calculate_heading_correction(yaw_err, dt_pid);

        long duty_a = (long)((duty_ratio - corr) * PWM_MAX_DUTY + 0.5f);
        long duty_b = (long)((duty_ratio + corr) * PWM_MAX_DUTY + 0.5f);
        set_motor_speed(duty_a, duty_b);

        /* exit when we've covered the distance */
        if (dist >= target_dist)
            break;

        last_dist = dist;
        last_ms = now_ms;
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }

    /* stop, coast & gather result */
    motor_stop();
    wait_for_stop(250, &result, target_yaw);
    return result;
}

void motor_set_direction(int motor, int direction)
{
    if (motor == MOTOR_RIGHT)
    {
        if (direction == MOTOR_FORWARD)
        {
            gpio_set_level(MOTOR_A_IN1, 1); // Input 1 HIGH
            gpio_set_level(MOTOR_A_IN2, 0); // Input 2 LOW
            // log_remote("Right motor set to FORWARD (IN1=1, IN2=0)");
        }
        else
        {
            gpio_set_level(MOTOR_A_IN1, 0); // Input 1 LOW
            gpio_set_level(MOTOR_A_IN2, 1); // Input 2 HIGH
            // log_remote("Right motor set to BACKWARD (IN1=0, IN2=1)");
        }
    }
    else
    {
        if (direction == MOTOR_FORWARD)
        {
            gpio_set_level(MOTOR_B_IN1, 1); // Input 3 HIGH
            gpio_set_level(MOTOR_B_IN2, 0); // Input 4 LOW
            // log_remote("Left motor set to FORWARD (IN1=1, IN2=0)");
        }
        else
        {
            gpio_set_level(MOTOR_B_IN1, 0); // Input 3 LOW
            gpio_set_level(MOTOR_B_IN2, 1); // Input 4 HIGH
            // log_remote("Left motor set to BACKWARD (IN1=0, IN2=1)");
        }
    }
}

void set_motor_turn(bool turn_right)
{
    if (turn_right)
    {
        // Right turn: right motor backward, left motor forward
        // log_remote("Setting RIGHT turn: Right motor backward, Left motor forward");
        motor_set_direction(MOTOR_RIGHT, MOTOR_BACKWARD);
        motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    }
    else
    {
        //   Left turn: right motor forward, left motor backward
        // log_remote("Setting LEFT turn: Right motor forward, Left motor backward");
        motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
        motor_set_direction(MOTOR_LEFT, MOTOR_BACKWARD);
    }
}

/**
 *  Slow, smooth forward drive for one "block".
 *  ──────  ────────────────────────────────────────────────────────────────────
 *  1.  Duty starts at 15 % and rises by 2 % every 50 ms while
 *      measured speed < 10 cm·s-¹  (or until the max cruise duty is hit).
 *  2.  Once cruising, the duty is capped by the linear-deceleration profile
 *      (same as in the fast routine) so the bot eases into the stop point.
 *  3.  Heading is kept with the same PID used in `motor_forward()`.
 *  4.  Instantaneous speed is logged (cm · s-¹).
 */
/******************************************************************************
 *  motor_rotate_to_cardinal_slow                                              *
 *                                                                             *
 *  Slow, accurate 90 ° (cardinal) turn with soft-start / soft-stop.           *
 *  Heading convention: 0° = N, positive = CLOCK-WISE.                         *
 ******************************************************************************/
void motor_turn(Direction target)
{
    float heading_offset = 0.0f;
    /* ── parameters (unchanged) ─────────────────────────────────────── */
    const float LEFT_FACTOR = 1.0f;
    const float RAMP_START_DUTY = 0.15f, RAMP_MAX_DUTY = 0.65f, RAMP_STEP = 0.025f;
    const float MIN_ANG_VEL_DPS = 20.0f;
    const float COARSE_TO_PROP_DEG = 50.0f, BRAKE_ZONE_BEGIN_DEG = 15.0f;
    const float MAX_PROP_DUTY = 0.60f, MAX_BRAKE_DUTY = 0.25f, OVERSHOOT_BRAKE_DUTY = 0.375f;
    const unsigned STABLE_TIME_REQUIRED_MS = 300, LOOP_PERIOD_MS = 15;
    /* ----------------------------------------------------------------- */

    current_direction = target;
    current_car.current_orientation = target;
    float desired_heading = normalize_angle((float)target * 90.0f + heading_offset);
    printf("[TURN_DEBUG] Desired heading: %.1f°\n", desired_heading);

    float heading = mpu_get_orientation().yaw;
    printf("[TURN_DEBUG] Current heading: %.1f°\n", heading);

    float error = calculate_yaw_error(desired_heading, heading);
    printf("[TURN_DEBUG] Initial error: %.1f°\n", error);

    /* **FIX** : with CW-positive headings, +error ⇒ CW ⇒ dir_sign = -1 */
    int dir_sign = (error >= 0.0f) ? -1 : +1;
    printf("[TURN_DEBUG] Turn direction: %s (dir_sign: %d)\n",
           dir_sign < 0 ? "CLOCKWISE" : "COUNTERCLOCKWISE", dir_sign);

    /* --------------- ramp-up ---------------------------------------- */
    float duty_ratio = RAMP_START_DUTY;
    float last_yaw = heading;
    unsigned long last_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (duty_ratio <= RAMP_MAX_DUTY)
    {
        long duty = (long)(duty_ratio * PWM_MAX_DUTY);
        set_motor_speed(dir_sign * duty, -dir_sign * duty * LEFT_FACTOR);

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));

        unsigned long now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = fmaxf((now_ms - last_ms) / 1000.0f, 0.001f);

        float yaw_rate = fabsf(calculate_yaw_error(mpu_get_orientation().yaw, last_yaw)) / dt;
        printf("[TURN_DEBUG] Ramp-up: duty=%.3f, yaw_rate=%.1f°/s\n", duty_ratio, yaw_rate);

        if (yaw_rate > MIN_ANG_VEL_DPS)
            break;

        duty_ratio = fminf(duty_ratio + RAMP_STEP, RAMP_MAX_DUTY);
        last_ms = now_ms;
        last_yaw = mpu_get_orientation().yaw;
    }
    const float cruise_duty = duty_ratio;
    printf("[TURN_DEBUG] Ramp-up complete, cruise duty: %.3f\n", cruise_duty);

    /* --------------- main loop -------------------------------------- */
    unsigned long stable_ms = 0;
    float previous_error = error;
    float prev_heading = heading;

    while (1)
    {
        unsigned long loop_start = xTaskGetTickCount() * portTICK_PERIOD_MS;

        heading = mpu_get_orientation().yaw;
        error = calculate_yaw_error(desired_heading, heading);
        float abs_err = fabsf(error);

        printf("[TURN_DEBUG] Loop: heading=%.1f°, error=%.1f°, abs_err=%.1f°\n",
               heading, error, abs_err);

        /* early exit if static & small error */
        if (fabsf(calculate_yaw_error(heading, prev_heading)) < 0.1f && abs_err < 10.0f)
        {
            printf("[TURN_DEBUG] Early exit: heading stable and error small\n");
            break;
        }
        prev_heading = heading;

        /* overshoot detection */
        if ((previous_error > 0 && error < 0) || (previous_error < 0 && error > 0))
        {
            printf("[TURN_DEBUG] Overshoot detected, applying brake\n");
            long brake = (long)(OVERSHOOT_BRAKE_DUTY * PWM_MAX_DUTY);
            set_motor_speed(dir_sign * brake, -dir_sign * brake * LEFT_FACTOR);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }

        /* duty profile */
        float cmd_ratio;
        if (abs_err >= COARSE_TO_PROP_DEG)
        {
            cmd_ratio = cruise_duty * dir_sign;
            printf("[TURN_DEBUG] Coarse control: cmd_ratio=%.3f\n", cmd_ratio);
        }
        else if (abs_err > BRAKE_ZONE_BEGIN_DEG)
        {
            float s = (abs_err - BRAKE_ZONE_BEGIN_DEG) /
                      (COARSE_TO_PROP_DEG - BRAKE_ZONE_BEGIN_DEG);
            cmd_ratio = MAX_PROP_DUTY * s * dir_sign;
            printf("[TURN_DEBUG] Proportional control: s=%.3f, cmd_ratio=%.3f\n", s, cmd_ratio);
        }
        else
        {
            float s = (BRAKE_ZONE_BEGIN_DEG - abs_err) / BRAKE_ZONE_BEGIN_DEG;
            cmd_ratio = -MAX_BRAKE_DUTY * s * dir_sign;
            printf("[TURN_DEBUG] Braking: s=%.3f, cmd_ratio=%.3f\n", s, cmd_ratio);
        }

        long duty = (long)(fabsf(cmd_ratio) * PWM_MAX_DUTY);
        set_motor_speed((cmd_ratio >= 0 ? 1 : -1) * duty,
                        -(cmd_ratio >= 0 ? 1 : -1) * duty * LEFT_FACTOR);

        /* stability test */
        if (abs_err < MOTOR_TOLERANCE_DEG)
        {
            stable_ms += LOOP_PERIOD_MS;
            printf("[TURN_DEBUG] Within tolerance, stable_ms=%lu\n", stable_ms);
            if (stable_ms >= STABLE_TIME_REQUIRED_MS)
            {
                printf("[TURN_DEBUG] Stability achieved, breaking\n");
                break;
            }
        }
        else
        {
            stable_ms = 0;
        }

        previous_error = error;
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }

    printf("[TURN_DEBUG] Turn complete, final heading: %.1f°\n", mpu_get_orientation().yaw);
    stop_motors();
}

void test_navigation(void)
{
    log_remote("Starting endless square navigation test");

    // Define the square sequence: North → East → South → West
    const Direction directions[] = {NORTH, EAST, SOUTH, WEST};
    const int NUM_DIRECTIONS = 4;
    int current_direction_index = 0;

    while (1)
    { // Endless loop
        Direction current_dir = directions[current_direction_index];
        Direction next_dir = directions[(current_direction_index + 1) % NUM_DIRECTIONS];

        // Drive forward one block
        log_remote("[Square Test] Driving %s for one block",
                   current_dir == NORTH ? "NORTH" : current_dir == EAST ? "EAST"
                                                : current_dir == SOUTH  ? "SOUTH"
                                                                        : "WEST");
        drive_result_t result = motor_forward();
        log_remote("[Square Test] Forward complete - Heading error: %.2f°, Front distance: %.1f cm",
                   result.heading, result.ultrasonic.front);

        // Wait a bit between movements
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Turn to next direction
        log_remote("[Square Test] Turning from %s to %s",
                   current_dir == NORTH ? "NORTH" : current_dir == EAST ? "EAST"
                                                : current_dir == SOUTH  ? "SOUTH"
                                                                        : "WEST",
                   next_dir == NORTH ? "NORTH" : next_dir == EAST ? "EAST"
                                             : next_dir == SOUTH  ? "SOUTH"
                                                                  : "WEST");
        motor_turn(next_dir);

        // Wait a bit between movements
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Move to next direction in sequence
        current_direction_index = (current_direction_index + 1) % NUM_DIRECTIONS;
    }
}