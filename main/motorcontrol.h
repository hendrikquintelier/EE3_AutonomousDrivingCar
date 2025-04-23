#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "exploration_algorithm/direction.h"

// Motor direction constants
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_FORWARD 0
#define MOTOR_BACKWARD 1

// Motor Control Parameters
#define MOTOR_SPEED 0.65f // 70% speed

// Turn Control Parameters
#define MOTOR_TIMEOUT 3000 // Maximum time allowed for a turn (3 seconds)

#define MOTOR_TOLERANCE 2.0f // Tolerance for turn completion in degrees
#define MOTOR_MIN_TIME 300   // Minimum time for a turn

// Timeout and tolerance constants
#define TURN_TIMEOUT_MS 5000     // Maximum time allowed for a turn (5 seconds)
#define MOTOR_TIMEOUT_MS 5000    // Maximum time allowed for motor operations (5 seconds)
#define MOTOR_TOLERANCE_DEG 6.0f // Tolerance for motor operations in degrees

// Duty cycle limits
#define MAX_DUTY_RATIO 0.95f // Maximum duty cycle ratio (95%)
#define MIN_DUTY_RATIO 0.1f  // Minimum duty cycle ratio (10%)

// Result structure for motor operations
typedef struct
{
    float heading;                    // Final heading after movement
    ultrasonic_readings_t ultrasonic; // Ultrasonic readings after movement
} drive_result_t;

// Initialize the motor control system
void motor_init(void);

// Drive forward at constant speed with yaw control
void motor_forward_constant_speed(float speed);

// Set motor direction
void motor_set_direction(int motor, int direction);

void motor_turn_to_cardinal(Direction target, float heading_offset);

// Slow variant of turn to cardinal function
void motor_turn_to_cardinal_slow(Direction target, float heading_offset);

// Stop all motors
void motor_stop(void);

// Updated function signature to accept compensation parameters and return results
drive_result_t motor_forward_distance(float heading_compensation, float distance_compensation);

// Control structures
typedef struct
{
    float integral;
    float last_error;
    float target_yaw;
    unsigned long last_update;
    bool is_turning;
} turn_control_t;

typedef struct
{
    float left_factor;
    float right_factor;
    bool is_calibrated;
} motor_calibration_t;

// Global variables
extern turn_control_t turn_control;
extern motor_calibration_t motor_calibration;

// Test motor directions
void test_motor_directions(void);

// Test functions
void test_motor_control(void);
void test_navigation(void);

#endif // MOTORCONTROL_H
