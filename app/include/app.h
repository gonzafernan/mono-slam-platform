/**
 * @file app.h
 * @brief Main application header file
 *
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Application initialization
 * @param imu Pointer to the IMU context
 * @param left_encoder Pointer to the left encoder context
 * @param right_encoder Pointer to the right encoder context
 * @return 0 on success, -1 on failure
 */
int app_init(void *imu, void *left_encoder, void *right_encoder);

/**
 * @brief Get the value of the left encoder
 * @return The current value of the left encoder in radians
 */
double app_get_left_encoder_value(void);

/**
 * @brief Get the value of the right encoder
 * @return The current value of the right encoder in radians
 */
double app_get_right_encoder_value(void);

#ifdef __cplusplus
}
#endif

#endif  // APP_H