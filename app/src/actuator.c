/**
 * @file actuator.c
 * @brief Actuator control module for handling actuator operations.
 */

#include "actuator.h"
#include <stddef.h>
#include "osal_port.h"

static void actuator_task(void *argument);

int actuator_init(actuator_t *actuator, void *task_attributes,
                  void *port_encoder, double counts_per_revolution) {
    actuator->task_handle = osal_task_static_create(
        actuator_task, (void *)actuator, task_attributes);
    return encoder_init(&actuator->encoder, port_encoder, counts_per_revolution,
                        0);
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