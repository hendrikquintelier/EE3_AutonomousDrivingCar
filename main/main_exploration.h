#ifndef MAIN_EXPLORATION_H
#define MAIN_EXPLORATION_H

#include <unistd.h>
#include "exploration_algorithm/globals.h"
#include "exploration_algorithm/algorithm_structs_PUBLIC/MapPoint.h"
#include "exploration_algorithm/algorithm_structs_PUBLIC/FundamentalPath.h"
#include "exploration_algorithm/algorithm_structs_PUBLIC/Path.h"
#include "exploration_algorithm/navigate.h"
#include "exploration_algorithm/Dijkstra.h"

// Global variable to track the previous MapPoint during exploration
extern MapPoint *former_map_point;

/**
 * @brief Checks if the car is currently at a MapPoint.
 *
 * @return int 1 if at a MapPoint, 0 otherwise
 */
int is_map_point();

/**
 * @brief Updates the ultrasonic sensor readings.
 */
void update_ultrasonic_readings(void);

/**
 * @brief Decides the next move based on ultrasonic sensor readings.
 */
void decide_next_move();

/**
 * @brief Selects the next unexplored MapPoint to visit.
 *
 * @return MapPoint* Pointer to the next MapPoint to explore, or NULL if none remain
 */
MapPoint *select_next_mappoint();

/**
 * @brief Handles navigation when revisiting an already discovered MapPoint.
 *
 * @param existing_point Pointer to the existing MapPoint
 */
void existing_map_point_algorithm(MapPoint *existing_point);

/**
 * @brief Checks if the track exploration has been successfully completed.
 *
 * @return int 1 if exploration is complete, 0 otherwise
 */
int checkValidTrackCompletion();

/**
 * @brief Starts the autonomous exploration of the track.
 */
void start_exploration(void);

/**
 * @brief Finds the shortest path to an unexplored MapPoint.
 *
 * @param current_point The current MapPoint
 * @return Path* Pointer to the shortest path, or NULL if no path found
 */
Path *find_shortest_path_to_mappoint_tbd(MapPoint *current_point);

#endif /* MAIN_EXPLORATION_H */
