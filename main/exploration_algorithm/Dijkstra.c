//
// Created by Hendrik Quintelier on 06/02/2025.
//
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "globals.h"
#include "algorithm_structs_PUBLIC/Path.h"

// ======================= PRIORITY QUEUE STRUCTURE ======================= //

/**
 * @struct PriorityQueueNode
 * @brief Represents a node in the priority queue used for Dijkstra's algorithm.
 */
typedef struct PriorityQueueNode {
    MapPoint *mapPoint;  /**< Pointer to the MapPoint */
    int cost;            /**< Cost associated with reaching this MapPoint */
    struct PriorityQueueNode *next; /**< Pointer to the next node in the queue */
} PriorityQueueNode;

// ======================= PRIORITY QUEUE FUNCTIONS ======================= //

/**
 * @brief Inserts a MapPoint into the priority queue (sorted by cost).
 *
 * @param head Pointer to the head of the priority queue.
 * @param mapPoint Pointer to the MapPoint being inserted.
 * @param cost The cost associated with reaching this MapPoint.
 */
void push(PriorityQueueNode **head, MapPoint *mapPoint, int cost) {
    PriorityQueueNode *newNode = malloc(sizeof(PriorityQueueNode));
    if (!newNode) {
        perror("Error: Memory allocation failed for PriorityQueueNode");
        exit(EXIT_FAILURE);
    }

    newNode->mapPoint = mapPoint;
    newNode->cost = cost;
    newNode->next = NULL;

    if (*head == NULL || cost < (*head)->cost) {
        newNode->next = *head;
        *head = newNode;
        return;
    }

    PriorityQueueNode *current = *head;
    while (current->next != NULL && current->next->cost <= cost) {
        current = current->next;
    }
    newNode->next = current->next;
    current->next = newNode;
}

/**
 * @brief Removes and returns the lowest-cost node from the priority queue.
 *
 * @param head Pointer to the head of the priority queue.
 * @return MapPoint* Pointer to the MapPoint with the lowest cost.
 */
MapPoint *pop(PriorityQueueNode **head) {
    if (*head == NULL) return NULL;

    PriorityQueueNode *temp = *head;
    MapPoint *mapPoint = temp->mapPoint;
    *head = temp->next;
    free(temp);

    return mapPoint;
}

// ======================= DIJKSTRA'S ALGORITHM ======================= //

/**
 * @brief Implements Dijkstra's Algorithm to find the shortest path to an unexplored MapPoint.
 *
 * @param current_map_point Pointer to the starting MapPoint.
 * @return Path* Pointer to the shortest path (caller must free memory).
 */
Path* find_shortest_path_to_mappoint_tbd(MapPoint *current_map_point) {
    if (!current_map_point) {
        fprintf(stderr, "Error: current_map_point is NULL\n");
        return NULL;
    }

    if (current_map_point->id < 0 || current_map_point->id >= num_map_points_all) {
        fprintf(stderr, "Error: Invalid MapPoint ID: %d (expected range: 0 to %d)\n",
                current_map_point->id, num_map_points_all - 1);
        return NULL;
    }

    // Allocate memory for distances and parent tracking arrays
    int *distances = malloc(num_map_points_all * sizeof(int));
    MapPoint **parents = malloc(num_map_points_all * sizeof(MapPoint *));
    if (!distances || !parents) {
        fprintf(stderr, "Error: Memory allocation failed\n");
        free(distances);
        free(parents);
        return NULL;
    }

    // Initialize distances and parent pointers
    for (int i = 0; i < num_map_points_all; i++) {
        distances[i] = INT_MAX;
        parents[i] = NULL;
    }

    // Priority queue initialization
    PriorityQueueNode *pq = NULL;
    push(&pq, current_map_point, 0);
    distances[current_map_point->id] = 0;

    MapPoint *closest_tbd = NULL;

    // === DIJKSTRA MAIN LOOP === //
    while (pq != NULL) {
        MapPoint *current = pop(&pq);

        // Prevent NULL pointer dereference
        if (!current) {
            continue;
        }

        // Check if the current MapPoint is unexplored
        for (int i = 0; i < num_map_points_tbd; i++) {
            if (map_points_tbd[i] == current) {
                closest_tbd = current;
                break;
            }
        }
        if (closest_tbd != NULL) break;

        // Expand neighbors (explore paths)
        for (int i = 0; i < current->numberOfPaths; i++) {
            FundamentalPath *path = &current->paths[i];
            if (!path->end) continue;

            int new_cost = distances[current->id] + path->distance;
            if (new_cost < distances[path->end->id]) {
                distances[path->end->id] = new_cost;
                parents[path->end->id] = current;
                push(&pq, path->end, new_cost);
            }
        }
    }

    // No reachable unexplored MapPoint found
    if (closest_tbd == NULL) {
        free(distances);
        free(parents);
        return NULL;
    }

    // Allocate memory for the shortest path
    Path *bestPath = malloc(sizeof(Path));
    if (!bestPath) {
        free(distances);
        free(parents);
        return NULL;
    }

    bestPath->start = current_map_point;
    bestPath->end = closest_tbd;
    bestPath->totalDistance = distances[closest_tbd->id];

    // === PATH RECONSTRUCTION === //
    int pathLength = 0;
    MapPoint *step = closest_tbd;

    // Count path length
    while (step != current_map_point) {
        if (!parents[step->id]) {
            free(distances);
            free(parents);
            free(bestPath);
            return NULL;
        }
        pathLength++;
        step = parents[step->id];
    }

    // Allocate memory for the route
    bestPath->route = malloc(pathLength * sizeof(FundamentalPath *));
    if (!bestPath->route) {
        free(bestPath);
        free(distances);
        free(parents);
        return NULL;
    }

    // Backtrack to construct the path
    int pathIndex = pathLength - 1;
    step = closest_tbd;

    while (step != current_map_point) {
        MapPoint *prev = parents[step->id];
        if (!prev) {
            free(bestPath->route);
            free(bestPath);
            free(distances);
            free(parents);
            return NULL;
        }

        for (int i = 0; i < prev->numberOfPaths; i++) {
            if (prev->paths[i].end == step) {
                bestPath->route[pathIndex] = &prev->paths[i];
                pathIndex--;
                break;
            }
        }
        step = prev;
    }

    // Free temporary arrays
    free(distances);
    free(parents);

    return bestPath;
}
