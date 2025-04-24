#include <stdio.h>
#include "globals.h"
#include "algorithm_structs_PUBLIC/Path.h"
#include "algorithm_structs_PUBLIC/MapPoint.h"
#include "../motorcontrol.h"
#include "../main_exploration.h"
#include "../track_navigation.h"
#include "wifi_logger.h"

/**
 * @brief Rotates the car towards an unexplored fundamental path at the given MapPoint.
 *
 * @param mp Pointer to the MapPoint structure.
 */
void turn_to_undiscovered_fundamental_path(MapPoint *mp)
{
    log_remote("[NAVIGATE] Attempting to turn to undiscovered path at MapPoint (%d, %d)\n",
               mp->location.x, mp->location.y);

    for (int i = 0; i < mp->numberOfPaths; i++)
    {
        log_remote("[NAVIGATE] Checking path %d: Direction %s, End: %s\n",
                   i + 1,
                   direction_to_string(mp->paths[i].direction),
                   mp->paths[i].end ? "Known" : "Unknown");

        // Check for an unexplored path
        if (mp->paths[i].end == NULL)
        {
            log_remote("[NAVIGATE] Found unexplored path in direction %s, initiating turn\n",
                       direction_to_string(mp->paths[i].direction));
            motor_turn(mp->paths[i].direction);
            break;
        }
    }
}

/**
 * @brief Rotates the car to face the specified direction.
 *
 * @param target_direction The desired orientation of the car.
 */
void rotate_to(Direction target_direction)
{
    current_car.current_orientation = target_direction;
}

/**
 * @brief Navigates the car along the given path.
 *
 * @param p Pointer to the Path structure containing the route.
 */
void navigate_path(const Path *p)
{
    log_remote("[NAVIGATE] Starting path navigation\n");

    // Validate the path before proceeding
    if (!p || !p->route || p->totalDistance == 0)
    {
        log_remote("[NAVIGATE] ERROR: Invalid path provided\n");
        return;
    }

    log_remote("[NAVIGATE] Path details - Total distance: %d, Start: (%d, %d), End: (%d, %d)\n",
               p->totalDistance,
               p->start->location.x, p->start->location.y,
               p->end->location.x, p->end->location.y);

    // Iterate through each step in the path
    for (int i = 0; i < p->totalDistance; i++)
    {
        FundamentalPath *step = p->route[i];
        log_remote("[NAVIGATE] Processing step %d/%d\n", i + 1, p->totalDistance);

        // Stop if the car has reached the final destination
        if (current_car.current_location.x == p->end->location.x &&
            current_car.current_location.y == p->end->location.y)
        {
            log_remote("[NAVIGATE] Reached final destination at (%d, %d)\n",
                       p->end->location.x, p->end->location.y);
            break;
        }

        // Validate the step before proceeding
        if (!step || !step->end)
        {
            log_remote("[NAVIGATE] ERROR: Invalid step %d in path\n", i + 1);
            return;
        }

        log_remote("[NAVIGATE] Current location: (%d, %d), Next step: Direction %s, Distance: %d\n",
                   current_car.current_location.x, current_car.current_location.y,
                   direction_to_string(step->direction), step->distance);

        // Rotate the car to align with the required direction
        log_remote("[NAVIGATE] Rotating to direction %s\n", direction_to_string(step->direction));
        motor_turn(step->direction);

        // Move forward along the path
        log_remote("[NAVIGATE] Moving forward %d units\n", step->distance);
        for (int j = 0; j < step->distance; j++)
        {
            move_forward();
        }

        // Update the car's position after completing the movement
        current_car.current_location = step->end->location;
        log_remote("[NAVIGATE] Moved to new location: (%d, %d)\n",
                   current_car.current_location.x, current_car.current_location.y);
    }

    // Adjust the car's orientation after reaching the final destination
    log_remote("[NAVIGATE] Path navigation complete, turning to undiscovered path\n");
    turn_to_undiscovered_fundamental_path(p->end);
}
