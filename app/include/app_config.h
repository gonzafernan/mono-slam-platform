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

#define ACTUATED_JOINTS_NUMBER 2  /// Number of joints in the robot platform
#define MAX_JOINT_NAME_LENGTH 32  /// Maximum length of joint names

#define IMU_I2C_ADDRESS ICM20948_I2C_ADDRESS1  /// I2C address for the IMU
#define IMU_I2C_TIMEOUT 1000                   /// Timeout for I2C operations
#define IMU_TASK_STACK_SIZE 512                /// Stack size for the IMU task9

#define MOTOR_GEAR_RATIO 30              /// Gear ratio for the motor
#define ENCODER_TICKS_PER_REVOLUTION 13  /// Encoder ticks per revolution
#define ENCODER_TICKS_MULTIPLIER 4  /// Multiplier for encoder ticks (rise/fall)

#define ACTUATOR_TASK_STACK_SIZE 512  /// Stack size for the actuator task

#define PLATFORM_WIDTH_MM 160.0  /// Platform width in milimeters
#define WHEEL_DIAMETER_MM 65.0   /// Wheel diameters in milimeters

#define LEFT_WHEEL_INDEX 0
#define LEFT_WHEEL_MOTOR_DIR 0
#define LEFT_WHEEL_ENCODER_SIGN -1

#define RIGHT_WHEEL_MOTOR_DIR 1
#define RIGHT_WHEEL_ENCODER_SIGN -1

/// @brief IMU task attributes
extern freertos_osal_task_static_attr_t imu_task_attr;

/// @brief IMU queue attributes
extern freertos_osal_queue_static_attr_t imu_queue_attr;

/// @brief Actuator 1 task attributes
extern freertos_osal_task_static_attr_t actuator1_task_attr;

/// @brief Actuator 1 state queue attributes
extern freertos_osal_queue_static_attr_t actuator1_state_queue_attr;

/// @brief Actuator 1 parameters queue attributes
extern freertos_osal_queue_static_attr_t actuator1_param_queue_attr;

/// @brief Actuator 2 task attributes
extern freertos_osal_task_static_attr_t actuator2_task_attr;

/// @brief Actuator 2 state queue attributes
extern freertos_osal_queue_static_attr_t actuator2_state_queue_attr;

/// @brief Actuator 2 parameters queue attributes
extern freertos_osal_queue_static_attr_t actuator2_param_queue_attr;

#ifdef __cplusplus
}
#endif

#endif  // APP_CONFIG_H
