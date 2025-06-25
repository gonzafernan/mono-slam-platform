/**
 * @file app_config.h
 * @brief Application configuration header file
 *
 * This file contains the configuration settings for the application.
 *
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "freertos_osal_port_config.h"
#include "icm20948_driver.h"

#define IMU_I2C_ADDRESS ICM20948_I2C_ADDRESS1  /// I2C address for the IMU
#define IMU_I2C_TIMEOUT 1000                   /// Timeout for I2C operations
#define IMU_TASK_STACK_SIZE 512                /// Stack size for the IMU task9

/// @brief IMU task attributes
extern freertos_osal_task_static_attr_t imu_task_attr;

/// @brief IMU queue attributes
extern freertos_osal_queue_static_attr_t imu_queue_attr;

#ifdef __cplusplus
}
#endif

#endif  // APP_CONFIG_H