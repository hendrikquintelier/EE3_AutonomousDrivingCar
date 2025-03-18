#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float left;
        float front;
        float right;
    } ultrasonic_readings_t;

    /**
     * @brief Initialize ultrasonic pins for left, front, right.
     */
    void ultrasonic_init(void);

    /**
     * @brief Returns all three distances in a single call.
     */
    ultrasonic_readings_t ultrasonic_get_all(void);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_H
