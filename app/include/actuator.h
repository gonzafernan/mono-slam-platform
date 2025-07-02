/**
 * @file actuator.h
 * @brief Header file for the actuator module
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"

typedef struct {
    void *task_handle;
    encoder_t encoder;  // Pointer to the first encoder
} actuator_t;

/**
 * @brief Initializes the actuator module.
 *
 * This function initializes the actuator hardware and prepares it for
 * operation. It should be called before any other actuator functions are used.
 *
 * @param actuator Pointer to the actuator structure.
 * @param task_attributes Pointer to the task attributes for the actuator task.
 * @param port_encoder Pointer to the encoder port context.
 * @param counts_per_revolution Number of counts per revolution for the encoder.
 * @return 0 on success, -1 on failure.
 */
int actuator_init(actuator_t *actuator, void *task_attributes,
                  void *port_encoder, double counts_per_revolution);

/**
 * @brief Get actuator last state.
 * This function retrieves the last recorded state of the actuator,
 * including the angular position and angular velocity.
 * @param actuator Pointer to the actuator structure.
 * @param angular_position Pointer to store the angular position in radians.
 * @param angular_velocity Pointer to store the angular velocity in radians per
 * second.
 */
void actuator_get_state(actuator_t *actuator, double *angular_position,
                        double *angular_velocity);

#ifdef __cplusplus
}
#endif

#endif  // ACTUATOR_H