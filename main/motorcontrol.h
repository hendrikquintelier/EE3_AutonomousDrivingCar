#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "surroundings.h"
#include "exploration_algorithm/direction.h"

#define MOTOR_SPEED 0.7f // 70% speed

// Initialize the motor control system
void motor_init(void);

// Drive forward at constant speed with yaw control
void motor_forward_constant_speed(float speed);



void motor_turn_to_cardinal(Direction target, float heading_offset);

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

#endif // MOTORCONTROL_H
