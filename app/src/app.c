/**
 * @file app.c
 * @brief Main application file
 *
 */

#include "app_config.h"
#include "imu.h"

int app_init(void *imu_context) {
    if (imu_init(imu_context, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        return -1;
    }
    return 0;
}