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
#include "hbridge_driver.h"

typedef struct {
    void *task_handle;
    encoder_t encoder;  // Pointer to the first encoder
    hbridge_t hbridge;  // Pointer to the H-bridge driver
} actuator_t;

typedef struct {
    void *port_encoder;            // Pointer to the encoder port context
    double counts_per_revolution;  // Number of counts per revolution
    void *port_hbridge_pwm;        // Pointer to the H-bridge PWM port context
    void *port_hbridge_in1;        // Pointer to the H-bridge IN1 port context
    void *port_hbridge_in2;        // Pointer to the H-bridge IN2 port context
} actuator_args_t;

/**
 * @brief Initializes the actuator module.
 *
 * This function initializes the actuator hardware and prepares it for
 * operation. It should be called before any other actuator functions are used.
 *
 * @param actuator Pointer to the actuator structure.
 * @param task_attributes Pointer to the task attributes for the actuator task.
 * @param args Pointer to the actuator arguments containing port contexts and
 * @return 0 on success, -1 on failure.
 */
int actuator_init(actuator_t *actuator, void *task_attributes,
                  actuator_args_t *args);

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