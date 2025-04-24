#include "exploration_algorithm/globals.h"
#include "exploration_algorithm/direction.h"
#include "motorcontrol.h"
#include "wifi_logger.h"
#include "exploration_algorithm/direction.h"
/**
 * @brief Moves the car forward if the next position is part of the track.
 *
 * The function calculates the new position based on the car's current orientation
 * and moves it only if the next position is a valid track or the start/finish line.
 */
void move_forward()
{
    motor_forward();
    int new_x = current_car.current_location.x;
    int new_y = current_car.current_location.y;

    // Log current orientation for debugging
    log_remote("[DEBUG] Current orientation: %d (NORTH=0°, EAST=90°, SOUTH=180°, WEST=270°)\n", current_car.current_orientation);

    // Determine next position based on direction
    switch (current_car.current_orientation)
    {
    case (Direction)NORTH: // 0°
        new_y -= 1;
        break;
    case (Direction)EAST: // 90°
        new_x += 1;
        break;
    case (Direction)SOUTH: // 180°
        new_y += 1;
        break;
    case (Direction)WEST: // 270°
        new_x -= 1;
        break;
    default:
        log_remote("Warning: Invalid car orientation detected (%d°). Unable to move forward.\n", current_car.current_orientation);
        return;
    }

    // Update car's current location
    current_car.current_location.x = new_x;
    current_car.current_location.y = new_y;
    log_remote("Car moved to new location: (%d, %d)\n", new_x, new_y);
}

/**
 * @brief Returns the opposite direction.
 *
 * @param dir The current direction.
 * @return Direction The opposite direction (NORTH ⇄ SOUTH, EAST ⇄ WEST).
 */
void turn_opposite_direction()
{
    Direction new_dir = (current_car.current_orientation + 2) % 4;
    motor_turn(new_dir);
    log_remote("[TURN] Turned to opposite direction: %s\n", direction_to_string(new_dir));
}

/**
 * @brief Returns the new direction after turning left (counterclockwise).
 *
 * @param dir The current direction.
 * @return Direction The new direction after a left turn.
 */
void turn_left()
{
    Direction new_dir = (current_car.current_orientation + 3) % 4;
    motor_turn(new_dir);
}

/**
 * @brief Returns the new direction after turning right (clockwise).
 *
 * @param dir The current direction.
 * @return Direction The new direction after a right turn.
 */
void turn_right()
{
    Direction new_dir = (current_car.current_orientation + 1) % 4;
    motor_turn(new_dir);
}
