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

#include "icm20948_driver.h"

#define IMU_I2C_ADDRESS ICM20948_I2C_ADDRESS1  /// I2C address for the IMU
#define IMU_I2C_TIMEOUT 1000                   /// Timeout for I2C operations

#ifdef __cplusplus
}
#endif

#endif  // APP_CONFIG_H