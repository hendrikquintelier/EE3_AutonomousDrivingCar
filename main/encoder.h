#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes both encoders (pins, ISRs).
     */
    void encoder_init(void);

    /**
     * @brief Returns the current count for a given encoder (1 or 2).
     */
    int encoder_get_count(int encoder_id);

    /**
     * @brief Resets the encoder count to 0.
     */
    void encoder_reset(int encoder_id);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
