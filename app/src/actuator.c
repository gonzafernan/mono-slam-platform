/**
 * @file actuator.c
 * @brief Actuator control module for handling actuator operations.
 */

#include "actuator.h"
#include <stddef.h>
#include "osal_port.h"

static void actuator_task(void *argument);

int actuator_init(actuator_t *actuator, void *task_attributes,
                  actuator_args_t *args) {
    actuator->task_handle = osal_task_static_create(
        actuator_task, (void *)actuator, task_attributes);
    if (encoder_init(&actuator->encoder, args->port_encoder,
                     args->counts_per_revolution, 0) < 0) {
        return -1;
    }
    if (hbridge_init(&actuator->hbridge, args->port_hbridge_pwm,
                     args->port_hbridge_in1, args->port_hbridge_in2) < 0) {
        return -1;
    }
    actuator->angular_velocity_setpoint = 0.0f;
    actuator->encoder_sign = args->encoder_sign;
    actuator->hbridge_dir = args->hbridge_dir;
    pid_init(&actuator->controller);
    pid_set_output_range(&actuator->controller, -100.0f, 100.0f);
    pid_set_integral_range(&actuator->controller, -100.0f, 100.0f);
    pid_set_kp(&actuator->controller, 0.0);
    pid_set_ki(&actuator->controller, 0.0);
    return 0;
}

static void actuator_task(void *argument) {
    actuator_t *actuator = (actuator_t *)argument;
    uint32_t time_ms;
    uint32_t last_exec_time;
    float output, angular_velocity;

    for (;;) {
        // osal_delay_until(&last_exec_time, 10);
        osal_delay(10);
        time_ms = osal_get_time_ms();
        encoder_sample(&actuator->encoder, time_ms);
        angular_velocity = actuator->encoder_sign *
                           encoder_get_angular_velocity(&actuator->encoder);
        output = pid_update(&actuator->controller, angular_velocity, 0.01);
        if (output < 0.0f) {
            hbridge_set_output(&actuator->hbridge, actuator->hbridge_dir, -output);
        } else {
            hbridge_set_output(&actuator->hbridge, actuator->hbridge_dir, output);
        }
    }
}

void actuator_get_state(actuator_t *actuator, double *angular_position,
                        double *angular_velocity) {
    *angular_position = actuator->encoder_sign *
                        encoder_get_angular_position(&actuator->encoder);
    *angular_velocity = actuator->encoder_sign *
                        encoder_get_angular_velocity(&actuator->encoder);
}

void actuator_update_setpoint(actuator_t *actuator, float angular_velocity) {
    pid_set_setpoint(&actuator->controller, angular_velocity);
}

void actuator_set_controller_kp(actuator_t *actuator, float kp) {
    pid_set_kp(&actuator->controller, kp);
}

void actuator_set_controller_ki(actuator_t *actuator, float ki) {
    pid_set_ki(&actuator->controller, ki);
}