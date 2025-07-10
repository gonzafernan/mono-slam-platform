/**
 * @file imu.c
 * @brief IMU data acquisition and processing
 */

#include <stdbool.h>
#include <stddef.h>

#include "ak09916_driver.h"
#include "app_config.h"
#include "gpio.h"
#include "icm20948_driver.h"
#include "imu.h"
#include "osal_port.h"

/// IMU singleton
static struct {
    icm20948_t accel_gyro;
    ak09916_t magnetometer;
    void *task_handle;
    void *queue_handle;
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
    imu.queue_handle =
        osal_queue_static_create(1, sizeof(imu_sample_t), &imu_queue_attr);
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

void imu_get_sample(imu_sample_t *sample, uint32_t timeout) {
    if (osal_queue_peek(imu.queue_handle, sample, timeout) < 0) {
        sample->accel_x = 0.0;
        sample->accel_y = 0.0;
        sample->accel_z = 0.0;
        sample->gyro_x = 0.0;
        sample->gyro_y = 0.0;
        sample->gyro_z = 0.0;
        sample->mag_x = 0.0;
        sample->mag_y = 0.0;
        sample->mag_z = 0.0;
    }
}

static void imu_task(void *argument) {
    imu_sample_t sample;

    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;

    for (;;) {
        // wait notification from imu (accel/gyro) data ready
        // osal_task_notify_wait(OSAL_MAX_DELAY);
        osDelay(100);
        if (imu_read_accelerometer(&accel_x, &accel_y, &accel_z)) {
            sample.accel_x = 0.0;
            sample.accel_y = 0.0;
            sample.accel_z = 0.0;
        } else {
            sample.accel_x = (double)accel_x / 16384.0;
            sample.accel_y = (double)accel_y / 16384.0;
            sample.accel_z = (double)accel_z / 16384.0;
        }
        if (imu_read_gyroscope(&gyro_x, &gyro_y, &gyro_z)) {
            sample.gyro_x = 0.0;
            sample.gyro_y = 0.0;
            sample.gyro_z = 0.0;
        } else {
            sample.gyro_x = (double)gyro_x / 131.0;
            sample.gyro_y = (double)gyro_y / 131.0;
            sample.gyro_z = (double)gyro_z / 131.0;
        }
        if (imu_read_magnetometer(&mag_x, &mag_y, &mag_z)) {
            sample.mag_x = 0.0;
            sample.mag_y = 0.0;
            sample.mag_z = 0.0;
        } else {
            sample.mag_x = (double)mag_x / 4912.0;
            sample.mag_y = (double)mag_y / 4912.0;
            sample.mag_z = (double)mag_z / 4912.0;
        }
        osal_queue_overwrite(imu.queue_handle, &sample);
    }
}

void imu_task_notify_from_isr(void) {
    osal_task_notify_from_isr(imu.task_handle);
}