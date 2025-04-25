//
// Created by Hendrik Quintelier on 05/02/2025.
//

#include <stdio.h>
#include <stdlib.h>
#include "FundamentalPath.h"
#include "MapPoint.h"
#include "../direction.h"
#include "../globals.h"
#include "../wifi_logger.h"

// ======================= GLOBAL VARIABLES ======================= //

/**
 * @brief Static counter for generating unique FundamentalPath IDs.
 */
static int fundamental_path_counter = 0;

// ======================= FUNCTION IMPLEMENTATIONS ======================= //

/**
 * @brief Determines the Manhattan distance between two MapPoints.
 *
 * @param start Pointer to the starting MapPoint.
 * @param end Pointer to the ending MapPoint.
 * @return int The Manhattan distance, or -1 if an invalid path.
 */
int determine_distance_mappoints(MapPoint *start, MapPoint *end)
{
    if (!start || !end)
    {
        fprintf(stderr, "Error: Null MapPoint passed to determine_distance_mappoints\n");
        return -1;
    }

    // Calculate the absolute difference in x and y coordinates
    int delta_x = abs(end->location.x - start->location.x);
    int delta_y = abs(end->location.y - start->location.y);

    // Determine direction from start to end
    Direction direction = determine_direction(start, end);

    // Ensure movement is strictly horizontal or vertical
    if ((direction == NORTH || direction == SOUTH) && delta_x != 0)
        return -1;
    if ((direction == EAST || direction == WEST) && delta_y != 0)
        return -1;

    return delta_x + delta_y;
}

/**
 * @brief Determines the distance of a FundamentalPath using its endpoints.
 *
 * @param path Pointer to the FundamentalPath.
 * @return int The calculated distance.
 */
int determine_distance_fundamentalpath(FundamentalPath *path)
{
    return determine_distance_mappoints(path->start, path->end);
}

/**
 * @brief Initializes a new FundamentalPath.
 *
 * @param fp Pointer to the FundamentalPath structure to initialize.
 * @param start Pointer to the starting MapPoint.
 * @param distance Distance value for the path.
 * @param direction Direction of the path.
 */
void initialize_fundamental_path(FundamentalPath *fp, MapPoint *start, int distance, Direction direction)
{
    if (!fp)
    {
        perror("Error: Null pointer passed to initialize_fundamental_path");
        return;
    }

    // Validate direction
    if (direction == DEFAULT_DIRECTION || direction < NORTH || direction > WEST)
    {
        log_remote("Error: Invalid direction passed to initialize_fundamental_path\n");
        return;
    }

    fp->id = fundamental_path_counter++;
    fp->start = start;
    fp->end = NULL;
    fp->distance = distance;
    fp->direction = direction;
    fp->dead_end = false; // Initialize dead_end flag to false

    add_fundamental_path(fp);
}

/**
 * @brief Updates or adds a FundamentalPath between two MapPoints.
 *
 * @param current Pointer to the current MapPoint.
 * @param former Pointer to the previous MapPoint.
 */
void update_latest_fundamental_path(MapPoint *current, MapPoint *former)
{
    if (!current || !former)
    {
        fprintf(stderr, "Error: Null pointer passed to update_latest_fundamental_path\n");
        return;
    }

    log_remote("[PATH] Updating paths between MapPoint %d (%d,%d) and MapPoint %d (%d,%d)\n",
               former->id, former->location.x, former->location.y,
               current->id, current->location.x, current->location.y);

    // Determine directions between the two MapPoints
    Direction fc_direction = determine_direction(former, current);
    Direction cf_direction = get_opposite_direction(fc_direction);

    // Validate both directions
    if (fc_direction == DEFAULT_DIRECTION || fc_direction < NORTH || fc_direction > WEST ||
        cf_direction == DEFAULT_DIRECTION || cf_direction < NORTH || cf_direction > WEST)
    {
        log_remote("[PATH] Error: Invalid directions calculated between MapPoints\n");
        return;
    }

    log_remote("[PATH] Directions: former->current: %s, current->former: %s\n",
               direction_to_string(fc_direction), direction_to_string(cf_direction));

    // === Check if a path already exists from 'former' to 'current' === //
    log_remote("[PATH] Checking former->current paths (total: %d)\n", former->numberOfPaths);
    bool found_fc_path = false;
    for (int i = 0; i < former->numberOfPaths; ++i)
    {
        log_remote("[PATH]   Path %d: Direction %s\n", i + 1, direction_to_string(former->paths[i].direction));
        if (former->paths[i].direction == fc_direction)
        {
            log_remote("[PATH]   Found existing path in direction %s\n", direction_to_string(fc_direction));
            former->paths[i].end = current;
            former->paths[i].distance = determine_distance_fundamentalpath(&former->paths[i]);
            found_fc_path = true;
            break;
        }
    }

    if (!found_fc_path)
    {
        log_remote("[PATH] No existing path found in direction %s, creating new one\n", direction_to_string(fc_direction));
        // Create a new path
        FundamentalPath *newPath = malloc(sizeof(FundamentalPath));
        if (!newPath)
        {
            perror("Error: Failed to allocate memory for FundamentalPath");
            exit(EXIT_FAILURE);
        }

        initialize_fundamental_path(newPath, former, 0, fc_direction);
        newPath->end = current;
        newPath->distance = determine_distance_fundamentalpath(newPath);

        // Safely expand the former->paths array
        FundamentalPath *newPaths = realloc(former->paths, (former->numberOfPaths + 1) * sizeof(FundamentalPath));
        if (!newPaths)
        {
            perror("Error: Failed to allocate memory for FundamentalPaths");
            exit(EXIT_FAILURE);
        }
        former->paths = newPaths;
        former->paths[former->numberOfPaths++] = *newPath;
        free(newPath);
    }

    // === Check if a path already exists from 'current' to 'former' === //
    log_remote("[PATH] Checking current->former paths (total: %d)\n", current->numberOfPaths);
    bool found_cf_path = false;
    for (int i = 0; i < current->numberOfPaths; ++i)
    {
        log_remote("[PATH]   Path %d: Direction %s\n", i + 1, direction_to_string(current->paths[i].direction));
        if (current->paths[i].direction == cf_direction)
        {
            log_remote("[PATH]   Found existing path in direction %s\n", direction_to_string(cf_direction));
            current->paths[i].end = former;
            current->paths[i].distance = determine_distance_fundamentalpath(&current->paths[i]);
            found_cf_path = true;
            break;
        }
    }

    if (!found_cf_path)
    {
        log_remote("[PATH] No existing path found in direction %s, creating new one\n", direction_to_string(cf_direction));
        // Create a new path
        FundamentalPath *newPath = malloc(sizeof(FundamentalPath));
        if (!newPath)
        {
            perror("Error: Failed to allocate memory for FundamentalPath");
            exit(EXIT_FAILURE);
        }

        initialize_fundamental_path(newPath, current, 0, cf_direction);
        newPath->end = former;
        newPath->distance = determine_distance_fundamentalpath(newPath);

        // Safely expand the current->paths array
        FundamentalPath *newPaths = realloc(current->paths, (current->numberOfPaths + 1) * sizeof(FundamentalPath));
        if (!newPaths)
        {
            perror("Error: Failed to allocate memory for FundamentalPaths");
            exit(EXIT_FAILURE);
        }
        current->paths = newPaths;
        current->paths[current->numberOfPaths++] = *newPath;
        free(newPath);
    }

    // Log the bidirectional connection
    log_remote("[PATH] Created/Updated bidirectional connection:\n");
    log_remote("  MapPoint %d (%d,%d) -> MapPoint %d (%d,%d) in direction %s\n",
               former->id, former->location.x, former->location.y,
               current->id, current->location.x, current->location.y,
               direction_to_string(fc_direction));
    log_remote("  MapPoint %d (%d,%d) -> MapPoint %d (%d,%d) in direction %s\n",
               current->id, current->location.x, current->location.y,
               former->id, former->location.x, former->location.y,
               direction_to_string(cf_direction));
}

/**
 * @brief Marks a fundamental path as a dead end.
 *
 * @param path Pointer to the FundamentalPath to mark as a dead end.
 */
void add_dead_end_flag_to_fundamental_path(FundamentalPath *path)
{
    if (!path)
    {
        fprintf(stderr, "Error: Null pointer passed to add_dead_end_flag_to_fundamental_path\n");
        return;
    }

    path->dead_end = true;
    log_remote("[DEAD END] Marked path %d as a dead end\n", path->id);
}
