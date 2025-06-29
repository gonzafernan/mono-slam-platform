/**
 * @file encoder.h
 * @brief Header file for the encoder module.
 */

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Initializes the encoder module.
 *
 * This function initializes the encoder hardware and prepares it for operation.
 * It should be called before any other encoder functions are used.
 *
 * @param encoder_handle Pointer to the encoder handle structure.
 * @return 0 on success, -1 on failure.
 */
int encoder_init(void *encoder_handle);

/**
 * @brief Samples the current encoder value.
 *
 * This function reads the current position of the encoder and returns it.
 *
 * @param encoder_handle Pointer to the encoder handle structure.
 * @return The current encoder value as a 32-bit unsigned integer.
 */
uint32_t encoder_sample(void *encoder_handle);

#ifdef __cplusplus
}
#endif

#endif  // ENCODER_H