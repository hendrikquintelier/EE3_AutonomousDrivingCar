#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdbool.h>

// Initialize the motor control system
void motor_init(void);

// Drive forward at constant speed with yaw control
void motor_forward_constant_speed(float speed);

// Turn 90 degrees left or right
void motor_turn_90(bool turn_right);

// Stop all motors
void motor_stop(void);

// Helper function declarations
static void normalize_angle(float *angle);
static void calibrate_motors(void);

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

#endif // MOTORCONTROL_H
