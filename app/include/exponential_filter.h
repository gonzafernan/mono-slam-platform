/**
 * @file exponential_filter.h
 * @brief Header for exponential filter module
 */

#ifndef EXPONENTIAL_FILTER_H
#define EXPONENTIAL_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha;
    float last_output;
} exponential_filter_t;

/**
 * @brief Exponential filter initialization
 * @param self Pointer to the exponential filter instance
 * @param alpha The filter smoothing factor betweeen 0 and 1
 */
void exponential_filter_init(exponential_filter_t *self, float alpha);

/**
 * @brief Exponential filter update
 * @param self Pointer to the exponential filter instance
 * @param input The new input to the filter
 */
float exponential_filter_update(exponential_filter_t *self, float input);

#ifdef __cplusplus
}
#endif

#endif  // EXPONENTIAL_FILTER_H