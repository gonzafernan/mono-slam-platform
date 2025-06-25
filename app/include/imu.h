/**
 * @file imu.h
 * @brief IMU data acquisition and processing
 *
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief IMU sample structure
 * This structure holds the data from the IMU sensors including
 * accelerometer, gyroscope, and magnetometer.
 */
typedef struct {
    double accel_x; /*!> Accelerometer X-axis data */
    double accel_y; /*!> Accelerometer Y-axis data */
    double accel_z; /*!> Accelerometer Z-axis data */
    double gyro_x;  /*!> Gyroscope X-axis data */
    double gyro_y;  /*!> Gyroscope Y-axis data */
    double gyro_z;  /*!> Gyroscope Z-axis data */
    double mag_x;   /*!> Magnetometer X-axis data */
    double mag_y;   /*!> Magnetometer Y-axis data */
    double mag_z;   /*!> Magnetometer Z-axis data */
} imu_sample_t;

/**
 * @brief IMU initialization
 * @param imu_context Pointer to the IMU context
 * @param device_address I2C device address of the IMU
 * @param timeout Timeout for I2C operations
 * @param task_attributes Pointer to the task attributes
 * @return 0 on success, -1 on failure
 */
int imu_init(void *imu_context, uint8_t device_address, uint32_t timeout,
             void *task_attributes);

/**
 * @brief Check if the IMU device is available
 * @return true if the IMU device is available, false otherwise
 */
bool imu_is_device_available(void);

/**
 * @brief Read accelerometer data from the IMU
 * @param accel_x Pointer to store the X-axis accelerometer data
 * @param accel_y Pointer to store the Y-axis accelerometer data
 * @param accel_z Pointer to store the Z-axis accelerometer data
 * @return 0 on success, -1 on failure
 */
int imu_read_accelerometer(int16_t *accel_x, int16_t *accel_y,
                           int16_t *accel_z);

/**
 * @brief Read gyroscope data from the IMU
 * @param gyro_x Pointer to store the X-axis gyroscope data
 * @param gyro_y Pointer to store the Y-axis gyroscope data
 * @param gyro_z Pointer to store the Z-axis gyroscope data
 * @return 0 on success, -1 on failure
 */
int imu_read_gyroscope(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/**
 * @brief Read magnetometer data from the IMU
 * @param mag_x Pointer to store the X-axis magnetometer data
 * @param mag_y Pointer to store the Y-axis magnetometer data
 * @param mag_z Pointer to store the Z-axis magnetometer data
 * @return 0 on success, -1 on failure
 */
int imu_read_magnetometer(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);

/**
 * @brief Notify the IMU task from an ISR (Interrupt Service Routine)
 * @note This function should be called from an ISR context
 */
void imu_task_notify_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif  // IMU_H