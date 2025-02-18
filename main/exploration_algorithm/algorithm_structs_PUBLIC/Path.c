//
// Created by Hendrik Quintelier on 05/02/2025.
//

#include <stdio.h>
#include "Path.h"

/**
 * @brief Initializes a Path structure between two MapPoints.
 *
 * @param path Pointer to the Path structure to be initialized.
 * @param start Pointer to the starting MapPoint.
 * @param end Pointer to the ending MapPoint.
 */
void initialize_path(Path *path, MapPoint *start, MapPoint *end) {
    if (!path || !start || !end) {
        perror("Error: Null pointer passed to initialize_path");
        return;
    }

    // Assign the start and end MapPoints
    path->start = start;
    path->end = end;
    path->totalDistance = 0;  // Default total distance

    // Placeholder for actual path-finding algorithm
    path->route = NULL;
}

/**
 * @brief Prints the details of a given Path.
 *
 * @param path Pointer to the Path structure to print.
 */
void printPathResult(const Path *path) {
    if (!path || !path->start || !path->end) {
        printf("❌ No valid path found.\n");
        return;
    }

    printf("\n===== Path Information =====\n");
    printf("Start MapPoint: ID %d, Location: (%d, %d)\n",
           path->start->id, path->start->location.x, path->start->location.y);

    printf("End MapPoint (Target): ID %d, Location: (%d, %d)\n",
           path->end->id, path->end->location.x, path->end->location.y);

    printf("Total Distance: %d\n", path->totalDistance);
    printf("================================\n");
}
