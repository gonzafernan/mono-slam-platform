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
    return 0;
}

static void actuator_task(void *argument) {
    actuator_t *actuator = (actuator_t *)argument;
    uint32_t timestamp = 0;

    for (;;) {
        osal_delay(100);
        timestamp += 100;
        encoder_sample(&actuator->encoder, timestamp);
    }
}

void actuator_get_state(actuator_t *actuator, double *angular_position,
                        double *angular_velocity) {
    *angular_position = encoder_get_angular_position(&actuator->encoder);
    *angular_velocity = encoder_get_angular_velocity(&actuator->encoder);
}