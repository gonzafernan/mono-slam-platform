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
 * @param imu_context Pointer to the IMU context
 * @return 0 on success, -1 on failure
 */
int app_init(void *imu_context);

#ifdef __cplusplus
}
#endif

#endif  // APP_H