/**
 * @file imu.c
 * @brief IMU data acquisition and processing
 */

#include <stdbool.h>
#include <stddef.h>

#include "ak09916_driver.h"
#include "gpio.h"
#include "icm20948_driver.h"
#include "osal_port.h"

/// IMU singleton
static struct {
    icm20948_t accel_gyro;
    ak09916_t magnetometer;
    void *task_handle;
} imu;

static void imu_task(void *argument);

int imu_init(void *imu_context, uint8_t device_address, uint32_t timeout,
             void *task_attributes) {
    if (icm20948_init(&imu.accel_gyro, imu_context, device_address, timeout) <
        0) {
        return -1;
    }
    if (ak09916_init(&imu.magnetometer, imu_context, timeout) < 0) {
        return -1;
    }
    imu.task_handle = osal_task_static_create(imu_task, NULL, task_attributes);
    return 0;
}

bool imu_is_device_available(void) {
    return icm20948_is_device_available(&imu.accel_gyro) &&
           ak09916_is_device_available(&imu.magnetometer);
}

int imu_read_accelerometer(int16_t *accel_x, int16_t *accel_y,
                           int16_t *accel_z) {
    return icm20948_read_accelerometer(&imu.accel_gyro, accel_x, accel_y,
                                       accel_z);
}

int imu_read_gyroscope(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    int status =
        icm20948_read_gyroscope(&imu.accel_gyro, gyro_x, gyro_y, gyro_z);
    icm20948_int_status_t int_status;
    icm20948_clear_int_status(&imu.accel_gyro, &int_status);
    return status;
}

int imu_read_magnetometer(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
    return ak09916_read_magnetometer(&imu.magnetometer, mag_x, mag_y, mag_z);
}

static void imu_task(void *argument) {
    for (;;) {
        // wait notification from imu (accel/gyro) data ready
        osal_task_notify_wait(OSAL_MAX_DELAY);
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    }
}

void imu_task_notify_from_isr(void) {
    osal_task_notify_from_isr(imu.task_handle);
}