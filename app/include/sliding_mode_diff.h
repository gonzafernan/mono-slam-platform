/**
 * @file sliding_mode_diff.h
 * @brief Header for Sliding Mode Differentiator module
 */

#ifndef SLIDING_MODE_DIFF_H
#define SLIDING_MODE_DIFF_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x1;
    float x2;
    float lambda1;
    float lambda2;
} sliding_mode1_diff_t;

/**
 * @brief Sliding mode differentiator of fisrt orden initialization.
 * @param self Pointer to sliding mode instance.
 * @param lambda1 Tunable parameter 1.
 * @param lambda1 Tunable parameter 2.
 */
void sliding_mode1_diff_init(sliding_mode1_diff_t *self, float lambda1,
                             float lambda2);

/**
 * @brief Sliding mode differentiator of fisrt orden update.
 * @param self Pointer to sliding mode instance.
 * @param x New input value.
 * @param dt Time elapsed between updates.
 */
void sliding_mode1_diff_update(sliding_mode1_diff_t *self, float x, float dt);

/**
 * @brief Get differentiated variable from sliding mode differentiator of fisrt
 * orden.
 * @param self Pointer to sliding mode instance.
 * @return Differentiated value.
 */
float sliding_model_diff_get_diff(sliding_mode1_diff_t *self);

#ifdef __cplusplus
}
#endif

#endif  // SLIDING_MODE_DIFF_H