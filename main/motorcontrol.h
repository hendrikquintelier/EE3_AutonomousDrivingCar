#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

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

#ifdef __cplusplus
}
#endif

#endif // MOTORCONTROL_H
