/**
 * @file app.c
 * @brief Main application file
 *
 */

#include "app.h"
#include <stdio.h>
#include "app_config.h"
#include "imu.h"

static struct {
    void *imu;            /*!> Pointer to the IMU context */
    actuator_t actuator1; /*!> Actuator 1 structure */
    actuator_t actuator2; /*!> Actuator 2 structure */
} robot_platform;

int app_init(void *imu, actuator_args_t *actuator1_args,
             actuator_args_t *actuator2_args) {
    robot_platform.imu = imu;

    if (imu_init(robot_platform.imu, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        printf("IMU initialization failed.\r\n");
        return -1;
    }
    actuator1_args->encoder_sign = (LEFT_WHEEL_INDEX == 0)
                                       ? LEFT_WHEEL_ENCODER_SIGN
                                       : RIGHT_WHEEL_ENCODER_SIGN;
    actuator1_args->hbridge_dir =
        (LEFT_WHEEL_INDEX == 0) ? LEFT_WHEEL_MOTOR_DIR : RIGHT_WHEEL_MOTOR_DIR;
    if (actuator_init(&robot_platform.actuator1, &actuator1_task_attr,
                      actuator1_args) < 0) {
        printf("Actuator 1 initialization failed.\r\n");
        return -1;
    }

    actuator2_args->encoder_sign = (LEFT_WHEEL_INDEX == 1)
                                       ? LEFT_WHEEL_ENCODER_SIGN
                                       : RIGHT_WHEEL_ENCODER_SIGN;
    actuator2_args->hbridge_dir =
        (LEFT_WHEEL_INDEX == 1) ? LEFT_WHEEL_MOTOR_DIR : RIGHT_WHEEL_MOTOR_DIR;
    if (actuator_init(&robot_platform.actuator2, &actuator2_task_attr,
                      actuator2_args) < 0) {
        printf("Actuator 2 initialization failed.\r\n");
        return -1;
    }
    printf("App initialized successfully.\r\n");
    return 0;
}

void app_get_joint_state(joint_state_t *joint_state) {
    uint8_t actuator1_index = (LEFT_WHEEL_INDEX == 0) ? 1 : 0;
    uint8_t actuator2_index = (LEFT_WHEEL_INDEX == 1) ? 1 : 0;
    actuator_get_state(&robot_platform.actuator1,
                       &joint_state->angular_position[actuator1_index],
                       &joint_state->angular_velocity[actuator1_index]);
    actuator_get_state(&robot_platform.actuator2,
                       &joint_state->angular_position[actuator2_index],
                       &joint_state->angular_velocity[actuator2_index]);
}

void app_update_joint_space_setpoint(float angular_velocity_left,
                                     float angular_velocity_right) {
    if (LEFT_WHEEL_INDEX == 0) {
        actuator_update_setpoint(&robot_platform.actuator1,
                                 angular_velocity_left);
        actuator_update_setpoint(&robot_platform.actuator2,
                                 angular_velocity_right);
    } else {
        actuator_update_setpoint(&robot_platform.actuator1,
                                 angular_velocity_right);
        actuator_update_setpoint(&robot_platform.actuator2,
                                 angular_velocity_left);
    }
}

void app_update_actuator_kp(float kp) {
    actuator_set_controller_kp(&robot_platform.actuator1, kp);
    actuator_set_controller_kp(&robot_platform.actuator2, kp);
}

void app_update_actuator_ki(float ki) {
    actuator_set_controller_ki(&robot_platform.actuator1, ki);
    actuator_set_controller_ki(&robot_platform.actuator2, ki);
}
