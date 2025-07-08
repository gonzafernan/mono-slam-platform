/**
 * @file pid.c
 * @brief PID controller module.
 */

#include "pid.h"

static float clamp(float value, float low_limit, float high_limit) {
    if (value < low_limit) {
        return low_limit;
    }
    if (value > high_limit) {
        return high_limit;
    }
    return value;
}

void pid_init(pid_controller_t *self) {
    self->kp = 0.0f;
    self->ki = 0.0f;
    self->kd = 0.0f;
    self->tau = 0.0f;

    self->min_output = 0.0f;
    self->max_output = 0.0;
    self->min_integral = 0.0f;
    self->max_integral = 0.0f;

    self->prev_input = 0.0f;
    self->setpoint = 0.0f;

    self->error_integral = 0.0f;
    self->input_derivative = 0.0f;
}

void pid_set_kp(pid_controller_t *self, float value) { self->kp = value; }

void pid_set_ki(pid_controller_t *self, float value) { self->ki = value; }

void pid_set_kd(pid_controller_t *self, float value) { self->kd = value; }

void pid_set_tau(pid_controller_t *self, float value) { self->tau = value; }

void pid_set_setpoint(pid_controller_t *self, float value) {
    self->setpoint = value;
}

void pid_set_output_range(pid_controller_t *self, float min_output,
                          float max_output) {
    self->min_output = min_output;
    self->max_output = max_output;
}

void pid_set_integral_range(pid_controller_t *self, float min_integral,
                            float max_integral) {
    self->min_integral = min_integral;
    self->max_integral = max_integral;
}

float pid_update(pid_controller_t *self, float input, float delta_time) {
    float error = self->setpoint - input;

    float proportional = self->kp * error;
    self->error_integral += delta_time * error;
    self->input_derivative =
        -(2.0f * self->kd * (input - self->prev_input) +
          (2.0f * self->tau - delta_time) * self->input_derivative) /
        (2.0f * self->tau + delta_time);

    // integral clamping
    self->error_integral =
        clamp(self->error_integral, self->min_integral, self->max_integral);

    float output =
        proportional + self->ki * self->error_integral + self->input_derivative;

    self->prev_input = input;

    return clamp(output, self->min_output, self->max_output);
}