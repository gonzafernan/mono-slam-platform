/**
 * @file app.c
 * @brief Main application file
 *
 */

#include "actuator.h"
#include "app_config.h"
#include "imu.h"

static struct {
    void *imu;  // Pointer to the IMU context
    actuator_t actuator1;
    actuator_t actuator2;
} robot_platform;

int app_init(void *imu, void *port_encoder1, void *port_encoder2) {
    robot_platform.imu = imu;
    double counts_per_revolution = MOTOR_GEAR_RATIO * ENCODER_TICKS_MULTIPLIER *
                                   ENCODER_TICKS_PER_REVOLUTION;

    if (imu_init(robot_platform.imu, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        return -1;
    }
    if (actuator_init(&robot_platform.actuator1, &actuator1_task_attr,
                      port_encoder1, counts_per_revolution) < 0) {
        return -1;
    }
    return 0;
}

double app_get_left_encoder_value(void) {
    return encoder_get_angular_position(&robot_platform.actuator1.encoder);
}

double app_get_left_encoder_velocity(void) {
    return encoder_get_angular_velocity(&robot_platform.actuator1.encoder);
}