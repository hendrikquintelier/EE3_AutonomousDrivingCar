#ifndef MPU_H
#define MPU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float yaw;
        float pitch;
        float roll;
        float ir_voltage;
    } mpu_data_t;

    /**
     * @brief Initializes I2C and configures MPU6050 for normal operation.
     */
    int mpu_init(void);

    /**
     * @brief Reads the current orientation (yaw, pitch, roll) from the MPU6050.
     * @return mpu_data_t with orientation data in degrees or radians.
     */
    mpu_data_t mpu_get_orientation(void);

    // Log orientation data with change rate
    void mpu_log_orientation(float current_yaw, float last_yaw, uint32_t print_interval_ms);

    /**
     * @brief Gets the filtered frontal distance from the ultrasonic sensor
     * @return float The filtered frontal distance in centimeters
     */
    float mpu_get_filtered_frontal_distance(void);

    /**
     * @brief Resets the movement tracking (position and velocity)
     */
    void mpu_reset_movement(void);

    static float read_ir_voltage(void);

#ifdef __cplusplus
}
#endif

#endif // MPU_H
