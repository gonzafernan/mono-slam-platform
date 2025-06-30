/**
 * @file encoder.h
 * @brief Header file for the encoder module.
 */

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *context;
    double gear_ratio;  // Gear ratio to output shaft
} encoder_t;

/**
 * @brief Initializes the encoder module.
 *
 * This function initializes the encoder hardware and prepares it for operation.
 * It should be called before any other encoder functions are used.
 *
 * @param encoder Pointer to the encoder structure.
 * @return 0 on success, -1 on failure.
 */
int encoder_init(encoder_t *encoder);

/**
 * @brief Samples the current encoder position.
 *
 * This function reads the current position of the encoder and returns it.
 *
 * @param encoder Pointer to the encoder structure.
 * @return The current encoder position in radians as a double.
 */
double encoder_sample_position(encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif  // ENCODER_H
