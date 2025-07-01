/**
 * @file encoder.c
 * @brief Encoder module for handling encoder operations.
 */

#include "encoder.h"
#include <math.h>
#include "encoder_port.h"

int encoder_init(encoder_t *encoder, void *context,
                 double counts_per_revolution, uint32_t timestamp) {
    if (encoder_port_init(context) < 0) {
        return -1;
    }
    encoder->context = context;
    encoder->counts_per_revolution = counts_per_revolution;
    encoder->last_sample = encoder_port_sample(encoder->context);
    encoder->accumulated_ticks = 0;
    encoder->last_timestamp = timestamp;
    encoder->last_angular_position = 0.0;
    encoder->last_angular_velocity = 0.0;
    return 0;
}

void encoder_sample(encoder_t *encoder, uint32_t timestamp) {
    uint16_t raw_value = encoder_port_sample(encoder->context);
    int16_t delta_ticks = (int16_t)(raw_value - encoder->last_sample);
    encoder->accumulated_ticks += delta_ticks;
    encoder->last_sample = raw_value;

    double angular_position = (double)encoder->accumulated_ticks * M_TWOPI /
                              encoder->counts_per_revolution;

    uint32_t delta_timestamp = timestamp - encoder->last_timestamp;

    if (delta_timestamp == 0) {
        // Avoid division by zero
        encoder->last_angular_position = angular_position;
        encoder->last_angular_velocity = 0.0;
        return;
    }

    encoder->last_angular_velocity =
        (angular_position - encoder->last_angular_position) /
        ((double)delta_timestamp / 1000.0);  // Convert ms to seconds
    encoder->last_angular_position = angular_position;
}

double encoder_get_angular_position(encoder_t *encoder) {
    return encoder->last_angular_position;
}

double encoder_get_angular_velocity(encoder_t *encoder) {
    return encoder->last_angular_velocity;
}