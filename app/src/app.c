/**
 * @file app.c
 * @brief Main application file
 *
 */

#include "app.h"
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
    if (actuator_init(&robot_platform.actuator2, &actuator2_task_attr,
                      port_encoder2, counts_per_revolution) < 0) {
        return -1;
    }
    return 0;
}

void app_get_joint_state(joint_state_t *joint_state) {
    actuator_get_state(&robot_platform.actuator1,
                       &joint_state->angular_position[0],
                       &joint_state->angular_velocity[0]);
    actuator_get_state(&robot_platform.actuator2,
                       &joint_state->angular_position[1],
                       &joint_state->angular_velocity[1]);
}