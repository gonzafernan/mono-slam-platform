/**
 * @file encoder.c
 * @brief Encoder module for handling encoder operations.
 */

#include "encoder.h"
#include <math.h>
#include "encoder_port.h"

int encoder_init(encoder_t *encoder) {
    if (encoder_port_init(encoder->context) < 0) {
        return -1;
    }
    encoder->last_sample = encoder_port_sample(encoder->context);
    encoder->accumulated_ticks = 0;
    return 0;
}

double encoder_sample_position(encoder_t *encoder) {
    uint16_t raw_value = encoder_port_sample(encoder->context);
    int16_t delta = (int16_t)(raw_value - encoder->last_sample);
    encoder->accumulated_ticks += delta;
    encoder->last_sample = raw_value;
    return (double)encoder->accumulated_ticks * M_TWOPI / encoder->counts_per_revolution;
}