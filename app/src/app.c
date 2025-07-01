/**
 * @file app.c
 * @brief Main application file
 *
 */

#include "app_config.h"
#include "encoder.h"
#include "imu.h"

static struct {
    void *imu;  // Pointer to the IMU context
    encoder_t encoder1;
    encoder_t encoder2;
} robot_platform;

int app_init(void *imu, void *port_encoder1, void *port_encoder2) {
    robot_platform.imu = imu;
    robot_platform.encoder1.context = port_encoder1;
    robot_platform.encoder1.counts_per_revolution =
        MOTOR_GEAR_RATIO * ENCODER_TICKS_MULTIPLIER *
        ENCODER_TICKS_PER_REVOLUTION;
    robot_platform.encoder2.context = port_encoder2;
    robot_platform.encoder2.counts_per_revolution =
        MOTOR_GEAR_RATIO * ENCODER_TICKS_MULTIPLIER *
        ENCODER_TICKS_PER_REVOLUTION;

    if (imu_init(robot_platform.imu, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        return -1;
    }
    if (encoder_init(&robot_platform.encoder1) < 0) {
        return -1;
    }
    if (encoder_init(&robot_platform.encoder2) < 0) {
        return -1;
    }
    return 0;
}

double app_get_left_encoder_value(void) {
    return encoder_sample_position(&robot_platform.encoder1);
}

double app_get_right_encoder_value(void) {
    return encoder_sample_position(&robot_platform.encoder2);
}