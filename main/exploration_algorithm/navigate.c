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
            former_map_point = mp;
            break;
        }
    }
}

/**
 * @brief Navigates the car along the given path.
 *
 * @param p Pointer to the Path structure containing the route.
 */
void navigate_path(const Path *p)
{
    log_remote("[NAVIGATE] Starting path navigation from (%d, %d) to (%d, %d)\n",
               current_car.current_location.x, current_car.current_location.y,
               p->end->location.x, p->end->location.y);

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
        log_remote("[NAVIGATE] Processing step %d/%d - Current location before step: (%d, %d)\n",
                   i + 1, p->totalDistance, current_car.current_location.x, current_car.current_location.y);

        // Stop if the car has reached the final destination
        if (current_car.current_location.x == p->end->location.x &&
            current_car.current_location.y == p->end->location.y)
        {
            log_remote("[NAVIGATE] Reached final destination at (%d, %d)\n",
                       p->end->location.x, p->end->location.y);
            break;
        }

        // Validate the step before proceeding
        if (!step)
        {
            log_remote("[NAVIGATE] ERROR: Invalid step %d in path\n", i + 1);
            return;
        }

        log_remote("[NAVIGATE] Step details - From: (%d, %d), To: (%d, %d), Direction: %s, Distance: %d\n",
                   step->start->location.x, step->start->location.y,
                   step->end->location.x, step->end->location.y,
                   direction_to_string(step->direction), step->distance);

        // Rotate the car to align with the required direction
        log_remote("[NAVIGATE] Rotating from %s to %s\n",
                   direction_to_string(current_car.current_orientation),
                   direction_to_string(step->direction));
        motor_turn(step->direction);

        // Move forward along the path
        log_remote("[NAVIGATE] Moving forward %d units\n", step->distance);
        for (int j = 0; j < step->distance; j++)
        {
            move_forward();
            log_remote("[NAVIGATE] Step %d/%d - Unit %d/%d - Current location: (%d, %d)\n",
                       i + 1, p->totalDistance, j + 1, step->distance,
                       current_car.current_location.x, current_car.current_location.y);
        }

        // Update the car's position after completing the movement
        Location prev_location = current_car.current_location;
        current_car.current_location = step->end->location;
        log_remote("[NAVIGATE] Step complete - Location updated from (%d, %d) to (%d, %d)\n",
                   prev_location.x, prev_location.y,
                   current_car.current_location.x, current_car.current_location.y);
    }

    // Adjust the car's orientation after reaching the final destination
    log_remote("[NAVIGATE] Path navigation complete at location (%d, %d)\n",
               current_car.current_location.x, current_car.current_location.y);
    turn_to_undiscovered_fundamental_path(p->end);
}
