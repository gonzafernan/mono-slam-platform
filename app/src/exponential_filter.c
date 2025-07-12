/**
 * @file exponential_filter.c
 * @brief Exponential filter
 */

#include "exponential_filter.h"
#include <math.h>

void exponential_filter_init(exponential_filter_t *self, float alpha) {
    self->alpha = alpha;
    self->last_output = nanf("");
}

float exponential_filter_update(exponential_filter_t *self, float input) {
    float output;
    if (isnanf(self->last_output)) {
        output = input;
    } else {
        output = self->alpha * input + (1 - self->alpha) * self->last_output;
    }
    self->last_output = output;
    return output;
}