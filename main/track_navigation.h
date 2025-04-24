#ifndef TRACK_NAVIGATION_H
#define TRACK_NAVIGATION_H

#include "exploration_algorithm/direction.h"

/**
 * @brief Moves the car forward if the next position is part of the track.
 */
void move_forward();

/**
 * @brief Turns the car to face the opposite direction.
 *
 * Updates the car's current orientation to face the opposite direction
 * (NORTH ⇄ SOUTH, EAST ⇄ WEST)
 */
void turn_opposite_direction();

/**
 * @brief Turns the car left (counterclockwise).
 *
 * Updates the car's current orientation to face the new direction
 */
void turn_left();

/**
 * @brief Turns the car right (clockwise).
 *
 * Updates the car's current orientation to face the new direction
 */
void turn_right();

#endif /* TRACK_NAVIGATION_H */
