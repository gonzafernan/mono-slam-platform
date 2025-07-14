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
#include "pid.h"

/**
 * @brief Actuator state sample structure
 * This structure holds the actuator state data.
 */
typedef struct {
    float angular_position; /*!> Actuator angular position */
    float angular_velocity; /*!> Actuator angular velocity */
} actuator_state_sample_t;

/**
 * @brief Actuator parameters structure
 * The structure holds the actuator parameters to be set.
 */
typedef struct {
    float kp;    /*!> Controller proportional gain */
    float ki;    /*!> Controller integral gain */
    float kd;    /*!> Controller derivative gain */
    float tau;   /*!> Controller derivative filter constant */
    float alpha; /*!> Encoder filter constant  */
} actuator_param_t;

typedef struct {
    void *task_handle;                // Actuator control task handle
    void *state_queue_handle;         // Queue state handle
    void *param_queue_handle;         // Queue state handle
    encoder_t encoder;                // Pointer to the first encoder
    hbridge_t hbridge;                // Pointer to the H-bridge driver
    int8_t encoder_sign;              // Encoder sign (-1, 1)
    uint8_t hbridge_dir;              // H-bridge direction (0, 1)
    pid_controller_t controller;      // Pointer to the actuator controller
    float angular_velocity_setpoint;  // Current angular velocity setpoint
} actuator_t;

typedef struct {
    void *port_encoder;            // Pointer to the encoder port context
    double counts_per_revolution;  // Number of counts per revolution
    void *port_hbridge_pwm;        // Pointer to the H-bridge PWM port context
    void *port_hbridge_in1;        // Pointer to the H-bridge IN1 port context
    void *port_hbridge_in2;        // Pointer to the H-bridge IN2 port context
    int8_t encoder_sign;           // Encoder sign (-1, 1)
    uint8_t hbridge_dir;           // H-bridge direction (0, 1)
    void *state_queue_attr;        // Pointer to actuator state queue attributes
    void *param_queue_attr;  // Pointer to actuator parameters queue attributes
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

/**
 * @brief Update actuator angular velocity setpoint.
 * @param actuator Pointer to the actuator structure.
 * @param angular_velocity New angular velocity setpoint.
 */
void actuator_update_setpoint(actuator_t *actuator, float angular_velocity);

/**
 * @brief Update actuator controller proportional gain
 * @param actuator Pointer to the actuator structure.
 * @param kp New proportional gain
 */
void actuator_set_controller_kp(actuator_t *actuator, float kp);

/**
 * @brief Update actuator controller integral gain
 * @param actuator Pointer to the actuator structure.
 * @param ki New integral gain
 */
void actuator_set_controller_ki(actuator_t *actuator, float ki);

#ifdef __cplusplus
}
#endif

#endif  // ACTUATOR_H