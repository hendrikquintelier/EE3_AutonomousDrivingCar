#ifndef MPU_H
#define MPU_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float yaw;
        float pitch;
        float roll;
    } mpu_data_t;

    /**
     * @brief Initializes I2C and configures MPU6050 for normal operation.
     */
    void mpu_init(void);

    /**
     * @brief Reads the current orientation (yaw, pitch, roll) from the MPU6050.
     * @return mpu_data_t with orientation data in degrees or radians.
     */
    mpu_data_t mpu_get_orientation(void);

#ifdef __cplusplus
}
#endif

#endif // MPU_H
