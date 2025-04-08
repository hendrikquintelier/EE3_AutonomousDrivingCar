#ifndef POSITION_COMPENSATION_H
#define POSITION_COMPENSATION_H

#include <stdbool.h>
#include <math.h>
#include "ultrasonic.h"

typedef enum
{
    Forward,
    Left,
    Right,
    Turn_Around
} Action;

typedef struct
{
    ultrasonic_readings_t ultrasonic;
    float forward_distance_overshoot;
    float heading;
} drive_result_t;

typedef struct
{
    Action action;
    float Xsideways; // #centimeters deviated sideways, right from the target center point. Negative means left from target center point.
    float Xforward;  // #centimeters over target distance, negative means before target center point.
    float heading;
} compensation_data_t;

typedef struct
{
    bool is_centered;
    float Xsideways;
    float Xforward;
    float left;
    float right;
    float front;
    bool left_track;
    bool right_track;
    bool front_track;
    float heading_error;
    float forward_distance_overshoot;
} surroundings_result_t;

typedef struct
{
    float heading_compensation;
    float distance_compensation;
} compensation_result_t;

/**
 * @brief Calculate the required adjustments to center the car in a block
 *
 * @param left_distance Distance to left wall in cm
 * @param front_distance Distance to front wall in cm
 * @param right_distance Distance to right wall in cm
 * @param heading Current heading in degrees
 * @return adjustment_result_t Structure containing adjustment requirements
 */
surroundings_result_t calculate_surroundings(drive_result_t data);

#endif // POSITION_COMPENSATION_H
