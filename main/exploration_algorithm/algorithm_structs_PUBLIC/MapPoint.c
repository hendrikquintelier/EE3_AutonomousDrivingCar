//
// Created by Hendrik Quintelier on 05/02/2025.
//

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "../globals.h"
#include "MapPoint.h"
#include "FundamentalPath.h"
#include "../direction.h"
#include "../wifi_logger.h"
#include "../main_exploration.h"

// ======================= GLOBAL VARIABLES ======================= //

/**
 * @brief Static counter for generating unique MapPoint IDs.
 */
static int map_point_counter = 0;

// ======================= MAPPOINT FUNCTIONS ======================= //

/**
 * @brief Initializes a new MapPoint with detected paths and adds it to the global array.
 *
 * @param mp Pointer to the MapPoint structure to be initialized.
 * @param location The location (x, y) of the MapPoint.
 * @param UltraSonicDetection Boolean array indicating detected paths (forward, left, right).
 */
void initialize_map_point(MapPoint *mp, Location location, bool UltraSonicDetection[3])
{
    if (!mp)
    {
        log_remote("Error: Null pointer passed to initialize_map_point\n");
        return;
    }

    // Assign a unique ID and store the location
    mp->id = map_point_counter++;
    mp->location = location;
    log_remote("Initializing MapPoint %d at location (%d, %d)\n", mp->id, location.x, location.y);

    // Count the number of detected paths with valid directions
    int pathCount = 0;
    Direction directions[3];
    bool validPaths[3] = {false, false, false};

    // First determine all directions
    for (int i = 0; i < 3; ++i)
    {
        if (UltraSonicDetection[i])
        {
            Direction dir;
            switch (i)
            {
            case 0:
                dir = (Direction)current_car.current_orientation;
                break;
            case 1:
                dir = get_orientation_on_the_left(current_car.current_orientation);
                break;
            case 2:
                dir = get_orientation_on_the_right(current_car.current_orientation);
                break;
            }

            // Only count paths with valid directions
            if (dir != DEFAULT_DIRECTION && dir >= NORTH && dir <= WEST)
            {
                directions[i] = dir;
                validPaths[i] = true;
                pathCount++;
            }
        }
    }

    mp->numberOfPaths = pathCount;
    log_remote("Number of valid paths detected: %d\n", pathCount);

    // Allocate memory for paths
    if (pathCount != 0)
    {
        mp->paths = malloc(pathCount * sizeof(FundamentalPath));
        if (!mp->paths)
        {
            log_remote("Error: Failed to allocate memory for FundamentalPaths\n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        // add path for dead ends!
        mp->paths = malloc(sizeof(FundamentalPath));
        mp->paths[0].direction = get_opposite_direction(current_car.current_orientation);
        mp->paths[0].end = former_map_point;
        mp->paths[0].distance = calculate_distance(mp->location, former_map_point->location);
    }

    // Initialize only the valid paths
    int pathIndex = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (validPaths[i])
        {
            initialize_fundamental_path(&mp->paths[pathIndex], mp, 0, directions[i]);
            log_remote("Path %d: Direction %s\n", pathIndex + 1, direction_to_string(directions[i]));
            pathIndex++;
        }
    }

    // Add to the global MapPoint array, resizing if necessary
    if (num_map_points_all == capacity_map_points_all)
    {
        capacity_map_points_all *= 2;
        map_points_all = realloc(map_points_all, capacity_map_points_all * sizeof(MapPoint));
        if (!map_points_all)
        {
            log_remote("Error: Failed to resize map_points_all array\n");
            exit(EXIT_FAILURE);
        }
    }
    map_points_all[num_map_points_all++] = mp;
    log_remote("MapPoint %d added to global array. Total MapPoints: %d\n", mp->id, num_map_points_all);

    // If the MapPoint has unexplored paths, add it to the "To Be Discovered" list
    if (mp_has_unexplored_paths(mp))
    {
        add_map_point_tbd(mp);
    }
}

/**
 * @brief Adds a MapPoint to the "To Be Discovered" list for future exploration.
 *
 * @param mp Pointer to the MapPoint to be added.
 */
void add_map_point_tbd(MapPoint *mp)
{
    if (num_map_points_tbd == capacity_map_points_tbd)
    {
        capacity_map_points_tbd *= 2;
        map_points_tbd = realloc(map_points_tbd, capacity_map_points_tbd * sizeof(MapPoint));
        if (!map_points_tbd)
        {
            perror("Error: Failed to expand map_points_tbd array");
            exit(EXIT_FAILURE);
        }
    }

    // Only avoid adding points at the start location
    if (mp->location.x != start.x || mp->location.y != start.y)
    {
        map_points_tbd[num_map_points_tbd++] = mp;
    }
}

/**
 * @brief Checks if a MapPoint has any paths with an unknown endpoint.
 *
 * @param mp Pointer to the MapPoint being checked.
 * @return int 1 if there are unexplored paths, otherwise 0.
 */
int mp_has_unexplored_paths(MapPoint *mp)
{
    for (int i = 0; i < mp->numberOfPaths; i++)
    {
        if (mp->paths[i].end == NULL)
        {
            return 1; // Found an unexplored path
        }
    }
    return 0;
}

/**
 * @brief Prints information about a given MapPoint.
 *
 * @param mp Pointer to the MapPoint to print.
 */
void print_map_point(const MapPoint *mp)
{
    if (!mp)
    {
        log_remote("Error: Null MapPoint passed to print_map_point.\n");
        return;
    }

    log_remote("\n========== MapPoint Info ==========\n");
    log_remote("ID: %d\n", mp->id);
    log_remote("Location: (%d, %d)\n", mp->location.x, mp->location.y);
    log_remote("Number of Paths: %d\n", mp->numberOfPaths);
    log_remote("------------------------------------\n");

    for (int i = 0; i < mp->numberOfPaths; ++i)
    {
        log_remote("  Path %d -> ", i + 1);
        if (mp->paths[i].end)
        {
            log_remote("Leads to MapPoint ID: %d, Distance: %d ", mp->paths[i].end->id, mp->paths[i].distance);
        }
        else
        {
            log_remote("Leads to: Unknown ");
        }
        log_remote("[Direction: %s]\n", direction_to_string(mp->paths[i].direction));
    }

    log_remote("====================================\n");
}

/**
 * @brief Checks if a MapPoint already exists at the car's current location.
 *
 * @return MapPoint* Pointer to the existing MapPoint, or NULL if not found.
 */
MapPoint *check_map_point_already_exists()
{
    // First check if we're at a MapPoint
    if (!is_map_point())
    {
        return NULL;
    }

    // Then check if a MapPoint already exists at this location
    for (int i = 0; i < num_map_points_all; i++)
    {
        if (map_points_all[i]->location.x == current_car.current_location.x &&
            map_points_all[i]->location.y == current_car.current_location.y)
        {
            return map_points_all[i];
        }
    }
    return NULL;
}

/**
 * @brief Calculates the Manhattan distance between two locations.
 *
 * @param a First location.
 * @param b Second location.
 * @return int The calculated Manhattan distance.
 */
int calculate_distance(Location a, Location b)
{
    return abs(a.x - b.x) + abs(a.y - b.y);
}

/**
 * @brief Updates an existing MapPoint and establishes a path to the latest added MapPoint.
 *
 * @param existing_point Pointer to the existing MapPoint.
 */
void update_existing_mappoint(MapPoint *existing_point)
{
    if (num_map_points_all < 2)
    {
        printf("Error: Not enough MapPoints to establish a connection.\n");
        return;
    }

    MapPoint *latest_point = map_points_all[num_map_points_all - 1];

    // Determine direction and distance
    Direction existing_to_latest = determine_direction(existing_point, latest_point);
    Direction latest_to_existing = get_opposite_direction(existing_to_latest);
    int distance = calculate_distance(existing_point->location, latest_point->location);

    // Search for existing paths and update if necessary
    int updated = 0;
    for (int i = 0; i < existing_point->numberOfPaths; i++)
    {
        if (existing_point->paths[i].direction == existing_to_latest)
        {
            existing_point->paths[i].end = latest_point;
            existing_point->paths[i].distance = distance;
            updated = 1;
            break;
        }
    }

    if (!updated)
    {
        // Allocate new paths dynamically
        existing_point->paths = realloc(existing_point->paths, (existing_point->numberOfPaths + 1) * sizeof(FundamentalPath));
        initialize_fundamental_path(&existing_point->paths[existing_point->numberOfPaths], existing_point, distance, existing_to_latest);
        existing_point->paths[existing_point->numberOfPaths].end = latest_point;
        existing_point->numberOfPaths++;
    }
}

/**
 * @brief Logs all map points and their connections to the remote logger.
 */
void log_all_map_points()
{
    log_remote("\n===== Current Map Points =====\n");
    log_remote("Total Map Points: %d\n", num_map_points_all);
    log_remote("Map Points To Be Discovered: %d\n", num_map_points_tbd);

    for (int i = 0; i < num_map_points_all; i++)
    {
        MapPoint *mp = map_points_all[i];
        log_remote("\nMap Point %d at (%d, %d) (Total paths: %d):\n",
                   mp->id, mp->location.x, mp->location.y, mp->numberOfPaths);

        for (int j = 0; j < mp->numberOfPaths; j++)
        {
            FundamentalPath *path = &mp->paths[j];
            log_remote("  Path %d/%d: Direction %s, ",
                       j + 1, mp->numberOfPaths, direction_to_string(path->direction));

            if (path->end)
            {
                log_remote("connects to Map Point %d (Distance: %d)\n",
                           path->end->id, path->distance);
            }
            else
            {
                log_remote("unexplored\n");
            }
        }
    }
    log_remote("\n=============================\n");
}