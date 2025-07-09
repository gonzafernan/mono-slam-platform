/**
 * @file sliding_mode_diff.c
 * @brief Sliding Mode Differentiator
 */

#include "sliding_mode_diff.h"
#include <math.h>

static inline float sign(float x) {
    // if (x < 0.0f) {
    //     return -1.0f;
    // }
    // if (x > 0.0f) {
    //     return 1.0f;
    // }
    // return 0.0f;
    return tanhf(50 * x);
}

void sliding_mode1_diff_init(sliding_mode1_diff_t *self, float lambda1,
                             float lambda2) {
    self->lambda1 = lambda1;
    self->lambda2 = lambda2;
    self->x1 = 0.0f;
    self->x2 = 0.0f;
}

void sliding_mode1_diff_update(sliding_mode1_diff_t *self, float x, float dt) {
    float err = self->x1 - x;
    float dx1 = self->x2 - self->lambda1 * sqrtf(fabsf(err)) * sign(err);
    float dx2 = -self->lambda2 * sign(err);

    // euler integration
    self->x1 += dx1 * dt;
    self->x2 += dx2 * dt;
}

float sliding_model_diff_get_diff(sliding_mode1_diff_t *self) {
    return self->x2;
}