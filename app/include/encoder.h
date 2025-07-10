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
#include "sliding_mode_diff.h"

typedef struct {
    void *context;
    float counts_per_revolution;  // Counts per revolution
    uint16_t last_sample;         // Last sampled value
    int64_t accumulated_ticks;    // Accumulated ticks for position calculation
    uint32_t last_timestamp;      // Last timestamp for velocity calculation
    sliding_mode1_diff_t diff_filter;  // Pointer to angular velocity filter
    float last_angular_position;       // Last angular position in radians
    float last_angular_velocity;       // Angular velocity in radians per second
} encoder_t;

/**
 * @brief Initializes the encoder module.
 *
 * This function initializes the encoder hardware and prepares it for operation.
 * It should be called before any other encoder functions are used.
 *
 * @param encoder Pointer to the encoder structure.
 * @param context Pointer to the encoder hardware context.
 * @param counts_per_revolution Number of counts per revolution for the encoder.
 * @param timestamp Initial timestamp for the encoder in miliseconds.
 * @return 0 on success, -1 on failure.
 */
int encoder_init(encoder_t *encoder, void *context, float counts_per_revolution,
                 uint32_t timestamp);

/**
 * @brief Samples the encoder and updates its angular position and velocity.
 *
 * This function reads the current encoder value, calculates the angular
 * position and velocity, and updates the encoder structure accordingly.
 *
 * @param encoder Pointer to the encoder structure.
 * @param timestamp Current timestamp for the encoder in miliseconds.
 */
void encoder_sample(encoder_t *encoder, uint32_t timestamp);

/**
 * @brief Gets the last angular position of the encoder.
 *
 * This function retrieves the last calculated angular position of the encoder.
 *
 * @param encoder Pointer to the encoder structure.
 * @return The last angular position in radians.
 */
float encoder_get_angular_position(encoder_t *encoder);

/**
 * @brief Gets the last angular velocity of the encoder.
 *
 * This function retrieves the last calculated angular velocity of the encoder.
 *
 * @param encoder Pointer to the encoder structure.
 * @return The last angular velocity in radians per second.
 */
float encoder_get_angular_velocity(encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif  // ENCODER_H
