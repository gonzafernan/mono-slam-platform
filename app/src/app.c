/**
 * @file app.c
 * @brief Main application file
 *
 */

#include "app_config.h"
#include "encoder.h"
#include "imu.h"

static struct {
    void *imu;            // Pointer to the IMU context
    void *left_encoder;   // Pointer to the left encoder context
    void *right_encoder;  // Pointer to the right encoder context
} robot_platform;

int app_init(void *imu, void *left_encoder, void *right_encoder) {
    robot_platform.imu = imu;
    robot_platform.left_encoder = left_encoder;
    robot_platform.right_encoder = right_encoder;

    if (imu_init(robot_platform.imu, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        return -1;
    }
    encoder_init(robot_platform.left_encoder);
    encoder_init(robot_platform.right_encoder);
    return 0;
}

uint32_t app_get_left_encoder_value(void) {
    return encoder_sample(robot_platform.left_encoder);
}

uint32_t app_get_right_encoder_value(void) {
    return encoder_sample(robot_platform.right_encoder);
}