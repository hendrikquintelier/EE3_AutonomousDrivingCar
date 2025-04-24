//
// Created by Hendrik Quintelier on 05/02/2025.
//

#include <stdio.h>
#include <stdlib.h>
#include "FundamentalPath.h"
#include "MapPoint.h"
#include "../direction.h"
#include "../globals.h"

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
 */
void initialize_fundamental_path(FundamentalPath *fp, MapPoint *start, int distance)
{
    if (!fp)
    {
        perror("Error: Null pointer passed to initialize_fundamental_path");
        return;
    }

    fp->id = fundamental_path_counter++;
    fp->start = start;
    fp->end = NULL;
    fp->distance = distance;
    fp->direction = NORTH; // Default direction (updated later)

    add_fundamental_path(fp);
}

/**
 * @brief Initializes multiple FundamentalPaths based on UltraSonicDetection.
 *
 * @param UltraSonicDetection Boolean array representing detected paths.
 * @return FundamentalPath* Pointer to dynamically allocated paths array.
 */
FundamentalPath *initialize_fundamental_paths(bool UltraSonicDetection[3])
{
    int pathCount = 0;

    // Count the number of detected paths
    for (int i = 0; i < 3; ++i)
    {
        if (UltraSonicDetection[i])
            pathCount++;
    }

    // Allocate memory for the paths
    FundamentalPath *paths = malloc(pathCount * sizeof(FundamentalPath));
    if (!paths)
    {
        perror("Error: Failed to allocate memory for FundamentalPaths");
        exit(EXIT_FAILURE);
    }

    int pathIndex = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (UltraSonicDetection[i])
        {
            initialize_fundamental_path(&paths[pathIndex++], NULL, 0);
        }
    }

    return paths;
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

    // Determine directions between the two MapPoints
    Direction fc_direction = determine_direction(former, current);
    Direction cf_direction = get_opposite_direction(fc_direction);

    FundamentalPath *fc_pointer_fundamental_path = NULL;
    FundamentalPath *cf_pointer_fundamental_path = NULL;

    // === Check if a path already exists from 'former' to 'current' === //
    for (int i = 0; i < former->numberOfPaths; ++i)
    {
        if (former->paths[i].direction == fc_direction)
        {
            fc_pointer_fundamental_path = &former->paths[i];
            break;
        }
    }

    // If path exists, update its endpoint and distance
    if (fc_pointer_fundamental_path)
    {
        fc_pointer_fundamental_path->end = current;
        fc_pointer_fundamental_path->distance = determine_distance_fundamentalpath(fc_pointer_fundamental_path);
    }
    else
    {
        // Otherwise, create a new path
        fc_pointer_fundamental_path = malloc(sizeof(FundamentalPath));
        if (!fc_pointer_fundamental_path)
        {
            perror("Error: Failed to allocate memory for FundamentalPath");
            exit(EXIT_FAILURE);
        }

        initialize_fundamental_path(fc_pointer_fundamental_path, former, 0);
        fc_pointer_fundamental_path->end = current;
        fc_pointer_fundamental_path->direction = fc_direction;
        fc_pointer_fundamental_path->distance = determine_distance_fundamentalpath(fc_pointer_fundamental_path);

        // Safely expand the former->paths array
        FundamentalPath *newPaths = realloc(former->paths, (former->numberOfPaths + 1) * sizeof(FundamentalPath));
        if (!newPaths)
        {
            perror("Error: Failed to allocate memory for FundamentalPaths");
            exit(EXIT_FAILURE);
        }
        former->paths = newPaths;
    }

    // === Check if a path already exists from 'current' to 'former' === //
    for (int i = 0; i < current->numberOfPaths; ++i)
    {
        if (current->paths[i].direction == cf_direction)
        {
            cf_pointer_fundamental_path = &current->paths[i];
            break;
        }
    }

    // If path exists, update its endpoint and distance
    if (cf_pointer_fundamental_path)
    {
        cf_pointer_fundamental_path->end = former;
        cf_pointer_fundamental_path->distance = fc_pointer_fundamental_path->distance;
    }
    else
    {
        // Otherwise, create a new path
        cf_pointer_fundamental_path = malloc(sizeof(FundamentalPath));
        if (!cf_pointer_fundamental_path)
        {
            perror("Error: Failed to allocate memory for FundamentalPath");
            exit(EXIT_FAILURE);
        }

        initialize_fundamental_path(cf_pointer_fundamental_path, current, 0);
        cf_pointer_fundamental_path->end = former;
        cf_pointer_fundamental_path->direction = cf_direction;
        cf_pointer_fundamental_path->distance = fc_pointer_fundamental_path->distance;

        // Safely expand the current->paths array
        FundamentalPath *newPaths = realloc(current->paths, (current->numberOfPaths + 1) * sizeof(FundamentalPath));
        if (!newPaths)
        {
            perror("Error: Failed to allocate memory for FundamentalPaths");
            exit(EXIT_FAILURE);
        }
        current->paths = newPaths;
    }
}
