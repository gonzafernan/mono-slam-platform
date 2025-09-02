/**
 * @file encoder.c
 * @brief Encoder module for handling encoder operations.
 */

#include "encoder.h"
#include <math.h>
#include "encoder_port.h"

int encoder_init(encoder_t *encoder, void *context, float counts_per_revolution,
                 float filter_alpha) {
    if (encoder_port_init(context) < 0) {
        return -1;
    }
    encoder->context = context;
    encoder->counts_per_revolution = counts_per_revolution;
    encoder->last_sample = encoder_port_sample(encoder->context);
    encoder->accumulated_ticks = 0;
    encoder->last_angular_position = 0.0;
    encoder->last_angular_velocity = 0.0;
    exponential_filter_init(&encoder->diff_filter, filter_alpha);
    return 0;
}

void encoder_sample(encoder_t *encoder, float delta_time) {
    uint16_t raw_value = encoder_port_sample(encoder->context);
    int16_t delta_ticks =
        (int16_t)((int32_t)raw_value - (int32_t)encoder->last_sample);
    encoder->accumulated_ticks += delta_ticks;
    encoder->last_sample = raw_value;

    float angular_position = (float)encoder->accumulated_ticks * 2 * M_PI /
                             encoder->counts_per_revolution;

    if (delta_time <= 0.0) {
        // Avoid division by zero
        encoder->last_angular_position = angular_position;
        encoder->last_angular_velocity = 0.0;
        return;
    }

    encoder->last_angular_velocity =
        (angular_position - encoder->last_angular_position) / delta_time;
    encoder->last_angular_velocity = exponential_filter_update(
        &encoder->diff_filter, encoder->last_angular_velocity);
    encoder->last_angular_position = angular_position;
}

float encoder_get_angular_position(encoder_t *encoder) {
    return encoder->last_angular_position;
}

float encoder_get_angular_velocity(encoder_t *encoder) {
    return encoder->last_angular_velocity;
}
