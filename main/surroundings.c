#include "surroundings.h"
#include "wifi_logger.h"
#include <math.h>
#include "exploration_algorithm/direction.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BLOCK_WIDTH_CM 40.0f
#define MAX_HEADING_DEVIATION_DEG 10.0f
#define MIN_WALL_DISTANCE_CM 5.0f

surroundings_result_t calculate_surroundings(drive_result_t data)
{
    surroundings_result_t result = {
        .is_centered = false,
        .Xsideways = 0.0f,
        .Xforward = 0.0f,
        .left = 0.0f,
        .right = 0.0f,
        .front = 0.0f,
        .left_track = false,
        .right_track = false,
        .front_track = false,
        .heading_error = data.heading,
        .forward_distance_overshoot = data.forward_distance_overshoot};

    // First check if heading needs correction
    if (fabsf(data.heading) > MAX_HEADING_DEVIATION_DEG)
    {
        return result;
    }

    float Xleft = 0.0f;
    float Xright = 0.0f;
    float Xfront = 0.0f;
    float heading_rad = data.heading * (M_PI / 180.0);

    // Process left distance if valid
    if (data.ultrasonic.left > 0)
    {
        result.left = data.ultrasonic.left * cos(heading_rad);
        Xleft = fmod(result.left, 40);
    }

    // Process right distance if valid
    if (data.ultrasonic.right > 0)
    {
        result.right = data.ultrasonic.right * cos(heading_rad);
        Xright = fmod(result.right, 40);
    }

    // Process front distance if valid; otherwise mark forward offset as invalid (-1)
    if (data.ultrasonic.front > 0)
    {
        result.front = data.ultrasonic.front * cos(heading_rad);
        Xfront = fmod(result.front, 40);

        result.Xforward = 20 - (Xfront + 10);
    }
    else
    {
        result.Xforward = -1;
    }

    // Calculate sideways offset (positive means right of center). SIDEWAYS DISTANCE: POSTIVE MEANS RIGHT OF CENTER
    if (data.ultrasonic.left > 0 && data.ultrasonic.right > 0)
    {
        result.Xsideways = Xleft - Xright;
    }
    else if (data.ultrasonic.left > 0)
    {
        result.Xsideways = (Xleft + 5) - 20;
    }
    else if (data.ultrasonic.right > 0)
    {
        result.Xsideways = 20 - (Xright + 5);
    }
    else
    {
        result.Xsideways = 0;
    }

    // Determine track flags based on thresholds
    if (result.left - result.Xsideways > 25.0f)
    {
        result.left_track = true;
    }
    if (result.right + result.Xsideways > 25.0f)
    {
        result.right_track = true;
    }
    if (result.front > 40.0f)
    {
        result.front_track = true;
    }

    return result;
}

//  NOTE: angle to the right is positive, equally for heading compensation. Before applyin in the turn section it will be inverted to negative.
static compensation_result_t calculate_compensations(compensation_data_t data)
{
    compensation_result_t result = {
        .heading_compensation = 0.0f,
        .distance_compensation = 0.0f};

    switch (data.action)
    {
    case Forward:
        // For forward movement, compensate for sideways drift
        result.heading_compensation = data.heading;   // Scale factor for heading compensation
        result.distance_compensation = data.Xforward; // Scale factor for distance compensation
        break;

    case Left:
        // For left turns, compensate for existing right lean
        result.heading_compensation = data.heading + atan2(data.Xforward, 40.0f); // Use full heading error for compensation
        result.distance_compensation = 0.0f;                                      // No distance compensation needed for turns
        break;

    case Right:
        // For right turns, compensate for existing right lean
        result.heading_compensation = data.heading - atan2(data.Xforward, 40.0f); // Use full heading error for compensation
        result.distance_compensation = 0.0f;                                      // No distance compensation needed for turns
        break;

    case Turn_Around:
        // For 180-degree turns, compensate for existing heading error
        result.heading_compensation = -data.heading;   // Use half the heading error for 180 turn
        result.distance_compensation = -data.Xforward; // No distance compensation needed for turns
        break;
    }

    return result;
}
