/**
 * @file transport.h
 * @brief Transport layer header file
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the transport layer for IMU data.
 * @param context Pointer to the transport context
 */
void transport_imu_init(void *context);

/**
 * @brief Publish IMU data through the transport layer.
 * This function is called to publish the IMU data through the transport layer.
 */
void transport_imu_publish(void);

#ifdef __cplusplus
}
#endif

#endif  // TRANSPORT_H