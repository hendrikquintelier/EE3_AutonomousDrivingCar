//
// Created by Hendrik Quintelier on 05/02/2025.
//

#include "direction.h"
#include "algorithm_structs_PUBLIC/MapPoint.h"

/**
 * @brief Returns the opposite direction.
 *
 * @param dir The current direction.
 * @return Direction The opposite direction (NORTH ⇄ SOUTH, EAST ⇄ WEST).
 */
Direction opposite_direction(Direction dir) {
    return (dir + 2) % 4;
}

/**
 * @brief Returns the new direction after turning left (counterclockwise).
 *
 * @param dir The current direction.
 * @return Direction The new direction after a left turn.
 */
Direction turn_left(Direction dir) {
    return (dir + 3) % 4;
}

/**
 * @brief Returns the new direction after turning right (clockwise).
 *
 * @param dir The current direction.
 * @return Direction The new direction after a right turn.
 */
Direction turn_right(Direction dir) {
    return (dir + 1) % 4;
}

/**
 * @brief Converts a direction enum to a human-readable string.
 *
 * @param dir The direction to convert.
 * @return const char* The corresponding string representation.
 */
const char* direction_to_string(Direction dir) {
    switch (dir) {
        case NORTH: return "North";
        case EAST:  return "East";
        case SOUTH: return "South";
        case WEST:  return "West";
        default:    return "Unknown";
    }
}

/**
 * @brief Converts a direction enum to its corresponding symbol.
 *
 * @param dir The direction to convert.
 * @return char The corresponding symbol:
 *         '^' for NORTH, '>' for EAST, 'v' for SOUTH, '<' for WEST, '?' if invalid.
 */
char direction_to_symbol(Direction dir) {
    switch (dir) {
        case NORTH: return '^';
        case EAST:  return '>';
        case SOUTH: return 'v';
        case WEST:  return '<';
        default:    return '?';
    }
}

/**
 * @brief Determines the direction from a starting MapPoint to an ending MapPoint.
 *
 * @param start Pointer to the starting MapPoint.
 * @param end Pointer to the ending MapPoint.
 * @return Direction The determined direction (NORTH, EAST, SOUTH, WEST) if movement is straight,
 *                   otherwise returns INVALID_DIRECTION.
 */
Direction determine_direction(MapPoint *start, MapPoint *end) {
    // Ensure valid inputs
    if (!start || !end) return INVALID_DIRECTION;

    int dx = end->location.x - start->location.x;
    int dy = end->location.y - start->location.y;

    // Determine direction based on relative position
    if (dx > 0 && dy == 0) return EAST;   // Moving right
    if (dx < 0 && dy == 0) return WEST;   // Moving left
    if (dy > 0 && dx == 0) return SOUTH;  // Moving downward
    if (dy < 0 && dx == 0) return NORTH;  // Moving upward

    // If movement is diagonal or invalid, return an invalid direction
    return INVALID_DIRECTION;
}
