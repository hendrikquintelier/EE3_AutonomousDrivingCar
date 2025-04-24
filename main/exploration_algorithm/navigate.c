#include <stdio.h>
#include "globals.h"
#include "algorithm_structs_PUBLIC/Path.h"
#include "algorithm_structs_PUBLIC/MapPoint.h"
#include "../motorcontrol.h"
#include "exploration.h"
#include "../track_navigation.h"

/**
 * @brief Rotates the car towards an unexplored fundamental path at the given MapPoint.
 *
 * @param mp Pointer to the MapPoint structure.
 */
void turn_to_undiscovered_fundamental_path(MapPoint *mp)
{
    for (int i = 0; i < mp->numberOfPaths; i++)
    {
        // Check for an unexplored path
        if (mp->paths[i].end == NULL)
        {
            current_car.current_orientation = mp->paths[i].direction;
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
    // Validate the path before proceeding
    if (!p || !p->route || p->totalDistance == 0)
    {
        return;
    }

    // Iterate through each step in the path
    for (int i = 0; i < p->totalDistance; i++)
    {
        FundamentalPath *step = p->route[i];

        // Stop if the car has reached the final destination
        if (current_car.current_location.x == p->end->location.x &&
            current_car.current_location.y == p->end->location.y)
        {
            break;
        }

        // Validate the step before proceeding
        if (!step || !step->end)
        {
            return;
        }

        // Rotate the car to align with the required direction
        rotate_to(step->direction);

        // Move forward along the path
        for (int j = 0; j < step->distance; j++)
        {
            move_forward();
        }

        // Update the car's position after completing the movement
        current_car.current_location = step->end->location;
    }

    // Adjust the car's orientation after reaching the final destination
    turn_to_undiscovered_fundamental_path(p->end);
}
