#include "exploration.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "algorithm_structs_PUBLIC/MapPoint.h"
#include "algorithm_structs_PUBLIC/FundamentalPath.h"

#include "Dijkstra.h"
#include "navigate.h"
#include "algorithm_structs_PUBLIC/Path.h"

#include "globals.h"

int checkValidTrackCompletion();

MapPoint *former_map_point = NULL; // Keeps track of the previous MapPoint

/**
 * @brief Checks if the car is currently at a MapPoint.
 *
 * @return int 1 if at a MapPoint, 0 otherwise.
 */
int is_map_point()
{
    int options = 0;

    if (ultrasonic_sensors[1])
        options++; // Can move left
    if (ultrasonic_sensors[2])
        options++; // Can move right

    return (options > 0); // More than one option indicates a MapPoint
}

/**
 * @brief Decides the next move based on ultrasonic sensor readings.
 */
void decide_next_move()
{
    update_ultrasonic_sensors();

    if (ultrasonic_sensors[0])
    {
        move_forward(&current_car);
        return;
    }

    // If forward is blocked, try turning
    if (ultrasonic_sensors[1])
    {
        rotate_left(&current_car);
        move_forward(&current_car);
    }
    else if (ultrasonic_sensors[2])
    {
        rotate_right(&current_car);
        move_forward(&current_car);
    }
    else
    {
        // No valid moves, perform a U-turn
        rotate_right(&current_car);
        rotate_right(&current_car);
    }
}

/**
 * @brief Selects the next unexplored MapPoint.
 *
 * @return MapPoint* Pointer to the next MapPoint to explore, or NULL if none remain.
 */
MapPoint *select_next_mappoint()
{
    if (num_map_points_tbd == 0)
        return NULL;

    for (int i = 0; i < num_map_points_tbd; i++)
    {
        if (map_points_tbd[i] != NULL)
        {
            return map_points_tbd[i];
        }
    }
    return NULL;
}

/**
 * @brief Handles navigation when revisiting an already discovered MapPoint.
 *
 * @param existing_point Pointer to the existing MapPoint.
 */
void existing_map_point_algorithm(MapPoint *existing_point)
{
    update_existing_mappoint(existing_point);

    // Ensure a FundamentalPath exists between the former and current MapPoint
    if (former_map_point)
    {
        update_latest_fundamental_path(existing_point, former_map_point);
    }

    // Check for unexplored paths at the current MapPoint
    int unexplored_paths = 0;
    for (int i = 0; i < existing_point->numberOfPaths; i++)
    {
        if (existing_point->paths[i].end == NULL)
        {
            unexplored_paths++;
        }
    }

    check_mappoints_tbd();

    if (unexplored_paths > 0)
    {
        decide_next_move();
    }
    else
    {
        // Find shortest path to the next unexplored MapPoint
        Path *resulting_path = find_shortest_path_to_mappoint_tbd(existing_point);

        if (resulting_path)
        {
            navigate_path(resulting_path);

            // Free allocated memory
            free(resulting_path->route);
            free(resulting_path);
        }
    }
}

/**
 * @brief Starts the autonomous exploration of the track.
 */
void start_exploration()
{
    while (1)
    {
        print_grid(current_car);

        // Update sensor readings before each move
        update_ultrasonic_sensors();
        check_mappoints_tbd();

        // Check if the current position is a MapPoint
        if (is_map_point())
        {
            MapPoint *existing_point = check_map_point_already_exists();

            if (existing_point)
            {
                existing_map_point_algorithm(existing_point);
            }
            else
            {
                // Allocate memory for a new MapPoint
                MapPoint *new_map_point = malloc(sizeof(MapPoint));
                if (!new_map_point)
                {
                    perror("Memory allocation failed for MapPoint");
                    exit(EXIT_FAILURE);
                }

                // Set location based on the car's current position
                Location location = {current_car.current_location.x, current_car.current_location.y};

                // Initialize new MapPoint with sensor data
                initialize_map_point(new_map_point, location, ultrasonic_sensors);

                // Link with the previous MapPoint if it exists
                if (former_map_point)
                {
                    update_latest_fundamental_path(new_map_point, former_map_point);
                }

                // Update the former MapPoint tracker
                former_map_point = new_map_point;
            }
        }

        // Decide the next movement
        decide_next_move();

        // Stop when exploration is complete
        if (num_map_points_tbd == 0 && num_all_fundamental_paths != 0 && num_map_points_all > 1)
        {
            break;
        }

        if (checkValidTrackCompletion())
        {
            break;
        }

        usleep(500000); // Delay for realistic movement speed
    }
}

/**
 * @brief Checks if the track exploration has been successfully completed.
 *
 * @return int 1 if exploration is complete, 0 otherwise.
 */
int checkValidTrackCompletion()
{
    return (current_car.current_location.x == start.x &&
            current_car.current_location.y == start.y &&
            num_map_points_all > 1 &&
            current_car.current_orientation != opposite_direction(start_orientation));
}
