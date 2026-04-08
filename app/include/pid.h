/**
 * @file pid.h
 * @brief Header file for the PID controller module.
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Controller gains
    float kp;  /*!> Controller proportional gain */
    float ki;  /*!> Controller integral gain */
    float kd;  /*!> Controller derivative gain */
    float tau; /*!> Controller derivative low-pass filter constant */

    // Controller limits
    float min_output;   /*!> Controller minimum output */
    float max_output;   /*!> Controller maximum output */
    float min_integral; /*!> Controller minimum integral action */
    float max_integral; /*!> Controller maximum integral action */

    // Controller history
    float prev_input; /*!> Controller previous input */

    float setpoint; /*!> Controller current setpoint */

    float error_integral;   /*!> Controller integral of error */
    float input_derivative; /*!> Controller input derivative */
} pid_controller_t;

/**
 * @brief PID controller initialization.
 * @param self Pointer to the PID controller handle
 */
void pid_init(pid_controller_t *self);

/**
 * @brief Set controller proportional gain.
 * @param self Pointer to the PID controller handle
 * @param value New proportional gain
 */
void pid_set_kp(pid_controller_t *self, float value);

/**
 * @brief Set controller integral gain.
 * @param self Pointer to the PID controller handle
 * @param value New integral gain
 */
void pid_set_ki(pid_controller_t *self, float value);

/**
 * @brief Set controller derivative gain.
 * @param self Pointer to the PID controller handle
 * @param value New derivatice gain
 */
void pid_set_kd(pid_controller_t *self, float value);

/**
 * @brief Set controller low-pass filter constant.
 * @param self Pointer to the PID controller handle
 * @param value New low-pass filter constant
 */
void pid_set_tau(pid_controller_t *self, float value);

/**
 * @brief Set controller output range (output saturation).
 * @param self Pointer to the PID controller handle
 * @param min_output Minimum output value
 * @param max_output Maximum output value
 */
void pid_set_output_range(pid_controller_t *self, float min_output,
                          float max_output);

/**
 * @brief Set controller integral range (integral clamping).
 * @param self Pointer to the PID controller handle
 * @param min_integral Minimum integral error
 * @param max_integral Maximum integral error
 */
void pid_set_integral_range(pid_controller_t *self, float min_integral,
                            float max_integral);

/**
 * @brief Set controller setpoint.
 * @param self Pointer to the PID controller handle
 * @param value New setpoint value for the controller
 */
void pid_set_setpoint(pid_controller_t *self, float value);

/**
 * @brief Update controller with new input.
 * @param self Pointer to the PID controller handle
 * @param input New input value for the controller
 * @param delta_time Time elapsed since last update in seconds
 * @return Output value from the controller
 */
float pid_update(pid_controller_t *self, float input, float delta_time);

/**
 * @brief Reset PID controller state.
 * @param self Pointer to the PID controller handle
 */
void pid_reset(pid_controller_t *self);

#ifdef __cplusplus
}
#endif

#endif  // PID_H
