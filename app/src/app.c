/**
 * @file app.c
 * @brief Main application file
 *
 */

#include <stdio.h>

#include "app.h"
#include "app_config.h"
#include "imu.h"
#include "osal_port.h"
#include "transport.h"

static struct {
    void *transport;              /*!> Pointer to transport layer context */
    void *supervisor_task_handle; /*!> Supervisor task handle */
    void *imu;                    /*!> Pointer to the IMU context */
    actuator_t actuator1;         /*!> Actuator 1 structure */
    actuator_t actuator2;         /*!> Actuator 2 structure */
} robot_platform;

static void supervisor_task(void *argument);

int app_init(void *transport, void *imu, actuator_args_t *actuator1_args,
             actuator_args_t *actuator2_args) {
    robot_platform.transport = transport;
    if (transport_init(robot_platform.transport, (void *)&transport_task_attr) <
        0) {
        printf("Trasnport layer initialization failed.\r\n");
    }

    robot_platform.supervisor_task_handle = osal_task_static_create(
        supervisor_task, NULL, (void *)&supervisor_task_attr);

    robot_platform.imu = imu;
    if (imu_init(robot_platform.imu, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        printf("IMU initialization failed (line %d).\r\n", __LINE__);
        return -1;
    }
    actuator1_args->encoder_sign = (LEFT_WHEEL_INDEX == 0)
                                       ? LEFT_WHEEL_ENCODER_SIGN
                                       : RIGHT_WHEEL_ENCODER_SIGN;
    robot_platform.actuator1.label[0] = '1';
    if (actuator_init(&robot_platform.actuator1, &actuator1_task_attr,
                      actuator1_args) < 0) {
        printf("Actuator 1 initialization failed.\r\n");
        return -1;
    }

    actuator2_args->encoder_sign = (LEFT_WHEEL_INDEX == 1)
                                       ? LEFT_WHEEL_ENCODER_SIGN
                                       : RIGHT_WHEEL_ENCODER_SIGN;
    robot_platform.actuator2.label[0] = '2';
    if (actuator_init(&robot_platform.actuator2, &actuator2_task_attr,
                      actuator2_args) < 0) {
        printf("Actuator 2 initialization failed.\r\n");
        return -1;
    }
    printf("App initialized successfully.\r\n");
    return 0;
}

static void supervisor_task(void *argument) {
    for (;;) {
        osal_delay(500);
        // printf("ACT%s: SET: %f - IN %f - ERR %f\r\n",
        //        robot_platform.actuator1.label,
        //        robot_platform.actuator1.controller.setpoint,
        //        robot_platform.actuator1.encoder.last_angular_velocity,
        //        robot_platform.actuator1.controller.error_integral);
        // printf("ACT%s: SET: %f - IN %f - ERR %f\r\n",
        //        robot_platform.actuator2.label,
        //        robot_platform.actuator2.controller.setpoint,
        //        robot_platform.actuator2.encoder.last_angular_velocity,
        //        robot_platform.actuator2.controller.error_integral);
    }
}

void app_get_joint_state(joint_state_t *joint_state) {
    if (LEFT_WHEEL_INDEX == 0) {
        // actuator1 -> LEFT (index 0), actuator2 -> RIGHT (index 1)
        actuator_get_state(&robot_platform.actuator1,
                           &joint_state->angular_position[0],
                           &joint_state->angular_velocity[0]);
        actuator_get_state(&robot_platform.actuator2,
                           &joint_state->angular_position[1],
                           &joint_state->angular_velocity[1]);
    } else {
        // actuator1 -> RIGHT (index 1), actuator2 -> LEFT (index 0)
        actuator_get_state(&robot_platform.actuator1,
                           &joint_state->angular_position[1],
                           &joint_state->angular_velocity[1]);
        actuator_get_state(&robot_platform.actuator2,
                           &joint_state->angular_position[0],
                           &joint_state->angular_velocity[0]);
    }
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
