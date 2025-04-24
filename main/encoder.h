#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Initialize the encoder hardware and interrupts.
    void encoder_init(void);

    // Reset both encoder counts to zero.
    void encoder_reset(void);

    // Get the raw pulse count from Encoder 1.
    int32_t encoder_get_count1(void);

    // Get the raw pulse count from Encoder 2.
    int32_t encoder_get_count2(void);

    // Calculate and return the average traveled distance (in centimeters)
    // based on both encoders.
    float encoder_get_distance(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
