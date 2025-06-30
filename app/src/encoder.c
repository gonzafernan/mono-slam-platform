/**
 * @file encoder.c
 * @brief Encoder module for handling encoder operations.
 */

#include "encoder.h"
#include <math.h>
#include "encoder_port.h"

int encoder_init(encoder_t *encoder) {
    return encoder_port_init(encoder->context);
}

double encoder_sample_position(encoder_t *encoder) {
    uint32_t raw_value = encoder_port_sample(encoder->context);
    return (double)raw_value * M_TWOPI / encoder->gear_ratio;
}