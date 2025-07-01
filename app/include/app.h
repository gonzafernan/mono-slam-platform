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
 * @param port_encoder1 Pointer to the encoder 1 context
 * @param port_encoder2 Pointer to the encoder 2 context
 * @return 0 on success, -1 on failure
 */
int app_init(void *imu, void *port_encoder1, void *port_encoder2);

/**
 * @brief Get the value of the left encoder
 * @return The current value of the left encoder in radians
 */
double app_get_left_encoder_value(void);

/**
 * @brief Get the value of the left encoder angular velocity
 * @return The angular velocity of the left encoder in radians/second
 */
double app_get_left_encoder_velocity(void);

#ifdef __cplusplus
}
#endif

#endif  // APP_H