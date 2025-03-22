#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize motors with PWM, etc.
     */
    void motor_init(void);

    /**
     * @brief Drive straight at a given speed, using MPU data to keep heading.
     *        'speed' is 0..1.
     */
    void motor_drive_straight(float speed);

    /**
     * @brief Set the desired heading to maintain while driving straight.
     */
    void motor_set_desired_heading(float heading);

    /**
     * @brief Turn the robot by 'angle' degrees (positive=turn left?), at a given speed.
     *        Blocks until turn is complete.
     */
    void motor_turn_by_angle(float angle, float speed);

    /**
     * @brief Actively brake, using short-circuit or dynamic braking.
     *        Blocks until encoders read zero movement.
     */
    void motor_brake(void);

    /**
     * @brief Stop motors (no PWM).
     */
    void motor_stop(void);

    /**
     * @brief Move forward at half speed (basic function, no heading correction).
     */
    void motor_forward_half_speed(void);

    /**
     * @brief Get the current speed of the car (0..1).
     */
    float motor_get_current_speed(void);

    /**
     * @brief Get the current heading of the car in degrees.
     */
    float motor_get_current_heading(void);

    /**
     * @brief Check if the car is currently braking.
     */
    bool motor_is_braking(void);

    /**
     * @brief Run PID calibration routine to optimize heading control gains.
     *        The car will drive straight at 40% speed while collecting data
     *        and adjusting the PID parameters.
     */
    void motor_calibrate_pid(void);

#ifdef __cplusplus
}
#endif

#endif // MOTORCONTROL_H
