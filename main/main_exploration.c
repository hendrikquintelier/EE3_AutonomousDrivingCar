#include "ultrasonic.h"
#include "exploration_algorithm/globals.h"
#include "wifi_logger.h"
#include "exploration_algorithm/algorithm_structs_PUBLIC/FundamentalPath.h"
#include "exploration_algorithm/algorithm_structs_PUBLIC/MapPoint.h"
#include "exploration_algorithm/algorithm_structs_PUBLIC/Path.h"
#include "exploration_algorithm/direction.h"
#include "motorcontrol.h"
#include "exploration_algorithm/Dijkstra.h"
#include "exploration_algorithm/navigate.h"
#include "track_navigation.h"
// Define the global variable
MapPoint *former_map_point = NULL;

void update_ultrasonic_readings(void)
{
    ultrasonic_readings_t readings = ultrasonic_get_all();

    // Update raw distance readings
    front_distance_ultrasonic = readings.front;
    left_distance_ultrasonic = readings.left;
    right_distance_ultrasonic = readings.right;

    // Set path detection flags based on 40cm threshold
    ultrasonic_sensors[0] = (readings.front > 30.0f); // Front path
    ultrasonic_sensors[1] = (readings.left > 35.0f);  // Left path
    ultrasonic_sensors[2] = (readings.right > 35.0f); // Right path
}

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
    update_ultrasonic_readings();

    // Log ultrasonic values for debugging
    log_remote("[ULTRASONIC] Front: %.1f cm (Path: %s)\n", front_distance_ultrasonic, ultrasonic_sensors[0] ? "CLEAR" : "BLOCKED");
    log_remote("[ULTRASONIC] Left:  %.1f cm (Path: %s)\n", left_distance_ultrasonic, ultrasonic_sensors[1] ? "CLEAR" : "BLOCKED");
    log_remote("[ULTRASONIC] Right: %.1f cm (Path: %s)\n", right_distance_ultrasonic, ultrasonic_sensors[2] ? "CLEAR" : "BLOCKED");

    if (ultrasonic_sensors[0])
    {
        printf("[DECISION] Moving forward - path clear\n");
        move_forward();
        return;
    }

    // If forward is blocked, try turning
    if (ultrasonic_sensors[1])
    {
        printf("[DECISION] Turning left - path clear\n");
        turn_left();
        move_forward();
    }
    else if (ultrasonic_sensors[2])
    {
        printf("[DECISION] Turning right - path clear\n");
        turn_right();
        move_forward();
    }
    else
    {
        printf("[DECISION] All paths blocked - turning opposite direction\n");

        // Mark the current path as a dead end if we have a former map point
        if (former_map_point)
        {
            // Find the path that led us here
            Direction current_direction = current_car.current_orientation;
            for (int i = 0; i < former_map_point->numberOfPaths; i++)
            {
                if (former_map_point->paths[i].direction == current_direction)
                {
                    add_dead_end_flag_to_fundamental_path(&former_map_point->paths[i]);
                    break;
                }
            }
        }

        turn_opposite_direction();
        move_forward();
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
            current_car.current_orientation != get_opposite_direction(start_orientation));
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

    // Check if we're at the start point in the wrong direction
    if (existing_point->location.x == start.x &&
        existing_point->location.y == start.y &&
        current_car.current_orientation == get_opposite_direction(start_orientation))
    {
        log_remote("[NAVIGATE] At start point in wrong direction, turning to correct orientation\n");
        turn_opposite_direction();
        return;
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
    former_map_point = existing_point;
}

void start_exploration()
{
    initialize_globals();
    log_remote("Starting Automatic Exploration...\n");

    start = current_car.current_location;
    start_orientation = current_car.current_orientation;

    while (1)
    {
        update_ultrasonic_readings();
        check_mappoints_tbd();

        if (is_map_point())
        {
            log_remote("Map point detected\n");
            MapPoint *existing_point = check_map_point_already_exists();

            if (existing_point)
            {
                log_remote("Existing MapPoint found:\n");
                existing_map_point_algorithm(existing_point);
            }
            else
            {
                // Allocate memory for a new MapPoint
                MapPoint *new_map_point = malloc(sizeof(MapPoint));
                if (!new_map_point)
                {
                    log_remote("Memory allocation failed for new MapPoint");
                    exit(EXIT_FAILURE);
                }
                // Set location based on the car's current position
                Location location = {current_car.current_location.x, current_car.current_location.y};

                // Initialize new MapPoint with sensor data
                initialize_map_point(new_map_point, location, ultrasonic_sensors);
                log_remote("New MapPoint created:\n");

                // Link with the previous MapPoint if it exists
                if (former_map_point)
                {
                    update_latest_fundamental_path(new_map_point, former_map_point);
                }

                // Update the former MapPoint tracker
                former_map_point = new_map_point;
            }
            log_all_map_points();
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

        // Delay for realistic movement speed
    }
}