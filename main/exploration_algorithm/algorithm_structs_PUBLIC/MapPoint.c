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
void initialize_map_point(MapPoint *mp, Location location, bool UltraSonicDetection[3]) {
    if (!mp) {
        perror("Error: Null pointer passed to initialize_map_point");
        return;
    }

    // Assign a unique ID and store the location
    mp->id = map_point_counter++;
    mp->location = location;

    // Count the number of detected paths
    int pathCount = 0;
    for (int i = 0; i < 3; ++i) {
        if (UltraSonicDetection[i]) pathCount++;
    }
    mp->numberOfPaths = pathCount;

    // Allocate memory for paths
    mp->paths = malloc(pathCount * sizeof(FundamentalPath));
    if (!mp->paths) {
        perror("Error: Failed to allocate memory for FundamentalPaths");
        exit(EXIT_FAILURE);
    }

    // Initialize detected paths and assign corresponding directions
    int pathIndex = 0;
    for (int i = 0; i < 3; ++i) {
        if (UltraSonicDetection[i]) {
            initialize_fundamental_path(&mp->paths[pathIndex], mp, 0);

            // Assign direction based on sensor index
            switch (i) {
                case 0:
                    mp->paths[pathIndex].direction = (Direction) current_car.current_orientation;
                    break; // Forward
                case 1:
                    mp->paths[pathIndex].direction = turn_left(current_car.current_orientation);
                    break; // Left
                case 2:
                    mp->paths[pathIndex].direction = turn_right(current_car.current_orientation);
                    break; // Right
            }

            pathIndex++;
        }
    }

    // Add to the global MapPoint array, resizing if necessary
    if (num_map_points_all == capacity_map_points_all) {
        capacity_map_points_all *= 2;
        map_points_all = realloc(map_points_all, capacity_map_points_all * sizeof(MapPoint));
        if (!map_points_all) {
            perror("Error: Failed to resize map_points_all array");
            exit(EXIT_FAILURE);
        }
    }
    map_points_all[num_map_points_all++] = mp;

    // If the MapPoint has unexplored paths, add it to the "To Be Discovered" list
    if (mp_has_unexplored_paths(mp)) {
        add_map_point_tbd(mp);
    }
}

/**
 * @brief Adds a MapPoint to the "To Be Discovered" list for future exploration.
 *
 * @param mp Pointer to the MapPoint to be added.
 */
void add_map_point_tbd(MapPoint *mp) {
    if (num_map_points_tbd == capacity_map_points_tbd) {
        capacity_map_points_tbd *= 2;
        map_points_tbd = realloc(map_points_tbd, capacity_map_points_tbd * sizeof(MapPoint));
        if (!map_points_tbd) {
            perror("Error: Failed to expand map_points_tbd array");
            exit(EXIT_FAILURE);
        }
    }

    // Avoid adding the start point (ID 0)
    if (mp->id != 0) {
        map_points_tbd[num_map_points_tbd++] = mp;
    }
}

/**
 * @brief Checks if a MapPoint has any paths with an unknown endpoint.
 *
 * @param mp Pointer to the MapPoint being checked.
 * @return int 1 if there are unexplored paths, otherwise 0.
 */
int mp_has_unexplored_paths(MapPoint *mp) {
    for (int i = 0; i < mp->numberOfPaths; i++) {
        if (mp->paths[i].end == NULL) {
            return 1;  // Found an unexplored path
        }
    }
    return 0;
}

/**
 * @brief Prints information about a given MapPoint.
 *
 * @param mp Pointer to the MapPoint to print.
 */
void print_map_point(const MapPoint *mp) {
    if (!mp) {
        printf("Error: Null MapPoint passed to print_map_point.\n");
        return;
    }

    printf("\n========== MapPoint Info ==========\n");
    printf("ID: %d\n", mp->id);
    printf("Location: (%d, %d)\n", mp->location.x, mp->location.y);
    printf("Number of Paths: %d\n", mp->numberOfPaths);
    printf("------------------------------------\n");

    for (int i = 0; i < mp->numberOfPaths; ++i) {
        printf("  Path %d -> ", i + 1);
        if (mp->paths[i].end) {
            printf("Leads to MapPoint ID: %d, Distance: %d ", mp->paths[i].end->id, mp->paths[i].distance);
        } else {
            printf("Leads to: Unknown ");
        }
        printf("[Direction: %s]\n", direction_to_string(mp->paths[i].direction));
    }

    printf("====================================\n");
}

/**
 * @brief Checks if a MapPoint already exists at the car's current location.
 *
 * @return MapPoint* Pointer to the existing MapPoint, or NULL if not found.
 */
MapPoint *check_map_point_already_exists() {
    for (int i = 0; i < num_map_points_all; i++) {
        if (map_points_all[i]->location.x == current_car.current_location.x &&
            map_points_all[i]->location.y == current_car.current_location.y) {
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
int calculate_distance(Location a, Location b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

/**
 * @brief Updates an existing MapPoint and establishes a path to the latest added MapPoint.
 *
 * @param existing_point Pointer to the existing MapPoint.
 */
void update_existing_mappoint(MapPoint *existing_point) {
    if (num_map_points_all < 2) {
        printf("Error: Not enough MapPoints to establish a connection.\n");
        return;
    }

    MapPoint *latest_point = map_points_all[num_map_points_all - 1];

    // Determine direction and distance
    Direction existing_to_latest = determine_direction(existing_point, latest_point);
    Direction latest_to_existing = opposite_direction(existing_to_latest);
    int distance = calculate_distance(existing_point->location, latest_point->location);

    // Search for existing paths and update if necessary
    int updated = 0;
    for (int i = 0; i < existing_point->numberOfPaths; i++) {
        if (existing_point->paths[i].direction == existing_to_latest) {
            existing_point->paths[i].end = latest_point;
            existing_point->paths[i].distance = distance;
            updated = 1;
            break;
        }
    }

    if (!updated) {
        // Allocate new paths dynamically
        existing_point->paths = realloc(existing_point->paths, (existing_point->numberOfPaths + 1) * sizeof(FundamentalPath));
        initialize_fundamental_path(&existing_point->paths[existing_point->numberOfPaths], existing_point, distance);
        existing_point->paths[existing_point->numberOfPaths].end = latest_point;
        existing_point->paths[existing_point->numberOfPaths].direction = existing_to_latest;
        existing_point->numberOfPaths++;
    }
}
