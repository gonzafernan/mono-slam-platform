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
#define IMU_TASK_STACK_SIZE 512                /// Stack size for the IMU task

uint32_t
    imu_stack_buffer[IMU_TASK_STACK_SIZE];  /// Buffer for the IMU task stack
StaticTask_t imu_task_buffer;

freertos_osal_task_static_attr_t imu_task_attr = {
    .name = "imu_task",                 /// Name of the IMU task
    .stack_size = IMU_TASK_STACK_SIZE,  /// Stack size for the IMU task
    .priority = osPriorityNormal,       /// Priority of the IMU task
    .stack = imu_stack_buffer,          /// Pointer to the stack
    .cb_mem = &imu_task_buffer,         /// Pointer to the control block memory
};

#ifdef __cplusplus
}
#endif

#endif  // APP_CONFIG_H