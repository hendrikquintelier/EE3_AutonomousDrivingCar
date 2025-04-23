#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"
#include "wifi_logger.h"
#include "motorcontrol.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "exploration_algorithm/direction.h"

void app_main(void)
{
    // Initialize components
    printf("[INIT] Starting system initialization\n");
    wifi_logger_init();
    mpu_init();
    ultrasonic_init();
    motor_init();
    encoder_init();
    printf("[INIT] System initialization complete\n");

    // Wait for everything to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Set initial direction to North
    Direction current_direction = NORTH;
    printf("[NAV] Initial navigation direction set to NORTH\n");

    while (1)
    {
        // Get ultrasonic readings
        ultrasonic_readings_t readings = ultrasonic_get_all();
        printf("[SENSOR] Distance readings:\n");
        printf("  Front: %.1f cm (Threshold: 40.0 cm)\n", readings.front);
        printf("  Left:  %.1f cm (Threshold: 40.0 cm)\n", readings.left);
        printf("  Right: %.1f cm (Threshold: 40.0 cm)\n", readings.right);

        // Decision making based on sensor readings
        if (readings.front > 40.0f)
        {
            printf("[DECISION] Path clear ahead (%.1f cm > 40.0 cm)\n", readings.front);
            printf("[ACTION] Executing forward movement\n");
            drive_result_t result = motor_forward_distance(0.0f, 0.0f);
            printf("[RESULT] Movement completed:\n");
            printf("  Final heading error: %.2f° (Target: 0.0°)\n", result.heading);
            printf("  Final front distance: %.1f cm\n", result.ultrasonic.front);
        }
        else if (readings.left > 40.0f)
        {
            printf("[DECISION] Left path clear (%.1f cm > 40.0 cm)\n", readings.left);
            printf("[ACTION] Planning left turn\n");

            // Calculate new direction (current direction - 90 degrees)
            Direction new_direction = (Direction)(((int)current_direction + 3) % 4);
            printf("[TURN] Direction change required:\n");
            printf("  Current: %s (%.0f°)\n",
                   current_direction == NORTH ? "NORTH" : current_direction == EAST ? "EAST"
                                                      : current_direction == SOUTH  ? "SOUTH"
                                                                                    : "WEST",
                   current_direction * 90.0f);
            printf("  Target: %s (%.0f°)\n",
                   new_direction == NORTH ? "NORTH" : new_direction == EAST ? "EAST"
                                                  : new_direction == SOUTH  ? "SOUTH"
                                                                            : "WEST",
                   new_direction * 90.0f);

            printf("[ACTION] Executing left turn\n");
            motor_turn_to_cardinal_slow(new_direction, 0.0f);
            current_direction = new_direction;

            printf("[ACTION] Driving forward after turn\n");
            drive_result_t result = motor_forward_distance(0.0f, 0.0f);
            printf("[RESULT] Movement completed:\n");
            printf("  Final heading error: %.2f° (Target: 0.0°)\n", result.heading);
            printf("  Final front distance: %.1f cm\n", result.ultrasonic.front);
        }
        else if (readings.right > 40.0f)
        {
            printf("[DECISION] Right path clear (%.1f cm > 40.0 cm)\n", readings.right);
            printf("[ACTION] Planning right turn\n");

            // Calculate new direction (current direction + 90 degrees)
            Direction new_direction = (Direction)(((int)current_direction + 1) % 4);
            printf("[TURN] Direction change required:\n");
            printf("  Current: %s (%.0f°)\n",
                   current_direction == NORTH ? "NORTH" : current_direction == EAST ? "EAST"
                                                      : current_direction == SOUTH  ? "SOUTH"
                                                                                    : "WEST",
                   current_direction * 90.0f);
            printf("  Target: %s (%.0f°)\n",
                   new_direction == NORTH ? "NORTH" : new_direction == EAST ? "EAST"
                                                  : new_direction == SOUTH  ? "SOUTH"
                                                                            : "WEST",
                   new_direction * 90.0f);

            printf("[ACTION] Executing right turn\n");
            motor_turn_to_cardinal_slow(new_direction, 0.0f);
            current_direction = new_direction;

            printf("[ACTION] Driving forward after turn\n");
            drive_result_t result = motor_forward_distance(0.0f, 0.0f);
            printf("[RESULT] Movement completed:\n");
            printf("  Final heading error: %.2f° (Target: 0.0°)\n", result.heading);
            printf("  Final front distance: %.1f cm\n", result.ultrasonic.front);
        }
        else
        {
            printf("[DECISION] No clear path detected:\n");
            printf("  Front: %.1f cm <= 40.0 cm\n", readings.front);
            printf("  Left:  %.1f cm <= 40.0 cm\n", readings.left);
            printf("  Right: %.1f cm <= 40.0 cm\n", readings.right);
            printf("[ACTION] Planning 180-degree turn\n");

            // Calculate new direction (current direction + 180 degrees)
            Direction new_direction = (Direction)(((int)current_direction + 2) % 4);
            printf("[TURN] Direction change required:\n");
            printf("  Current: %s (%.0f°)\n",
                   current_direction == NORTH ? "NORTH" : current_direction == EAST ? "EAST"
                                                      : current_direction == SOUTH  ? "SOUTH"
                                                                                    : "WEST",
                   current_direction * 90.0f);
            printf("  Target: %s (%.0f°)\n",
                   new_direction == NORTH ? "NORTH" : new_direction == EAST ? "EAST"
                                                  : new_direction == SOUTH  ? "SOUTH"
                                                                            : "WEST",
                   new_direction * 90.0f);

            printf("[ACTION] Executing 180-degree turn\n");
            motor_turn_to_cardinal_slow(new_direction, 0.0f);
            current_direction = new_direction;

            printf("[ACTION] Driving forward after turn\n");
            drive_result_t result = motor_forward_distance(0.0f, 0.0f);
            printf("[RESULT] Movement completed:\n");
            printf("  Final heading error: %.2f° (Target: 0.0°)\n", result.heading);
            printf("  Final front distance: %.1f cm\n", result.ultrasonic.front);
        }

        // Small delay between iterations
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
