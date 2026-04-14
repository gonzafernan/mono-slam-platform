/**
 * @file app.c
 * @brief Main application file
 *
 */

#include <stdio.h>

#include "actuator.h"
#include "app.h"
#include "app_config.h"
#include "diff_drive.h"
#include "diff_drive_estimator.h"
#include "imu.h"
#include "osal_port.h"

static struct {
    void *supervisor_task_handle;       /*!> Supervisor task handle */
    void *state_estimation_task_handle; /*!> State estimation task handle */
    void *imu;                          /*!> Pointer to the IMU context */
    actuator_t actuator1;               /*!> Actuator 1 structure */
    actuator_t actuator2;               /*!> Actuator 2 structure */
    diff_drive_t diff_drive;    /*!> Differential drive kinematics state */
    pose_estimator_t estimator; /*!> Pose estimator interface */
    void *wheel_vel_setpoint_mailbox; /*!> Wheel velocity setpoint mailbox
                                         handle */
    void *odometry_mailbox;           /*!> Odometry output mailbox handle */
} robot_platform;

static void supervisor_task(void *argument);
static void state_estimation_task(void *argument);

int app_init(void *imu, actuator_args_t *actuator1_args,
             actuator_args_t *actuator2_args) {
    robot_platform.wheel_vel_setpoint_mailbox = osal_queue_static_create(
        1, sizeof(wheel_vel_setpoint_t), &wheel_vel_setpoint_mailbox_attr);

    robot_platform.odometry_mailbox = osal_queue_static_create(
        1, sizeof(pose_estimator_state_t), &odometry_mailbox_attr);

    robot_platform.supervisor_task_handle = osal_task_static_create(
        supervisor_task, NULL, (void *)&supervisor_task_attr);

    robot_platform.imu = imu;
    if (imu_init(robot_platform.imu, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT,
                 &imu_task_attr) < 0) {
        printf("IMU initialization failed.\r\n");
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

    diff_drive_init(&robot_platform.diff_drive, WHEEL_DIAMETER_MM,
                    PLATFORM_WIDTH_MM);

#ifdef POSE_ESTIMATOR_DIFF_DRIVE
    diff_drive_estimator_init(&robot_platform.estimator,
                              &robot_platform.diff_drive);
#endif

    robot_platform.state_estimation_task_handle = osal_task_static_create(
        state_estimation_task, NULL, (void *)&state_estimation_task_attr);

    printf("App initialized successfully.\r\n");
    return 0;
}

static void supervisor_task(void *argument) {
    wheel_vel_setpoint_t setpoint;
    for (;;) {
        int result =
            osal_queue_receive(robot_platform.wheel_vel_setpoint_mailbox,
                               &setpoint, ACTUATOR_WATCHDOG_TIMEOUT_MS);
        if (result != 0) {
            setpoint.angular_velocity_left = 0.0f;
            setpoint.angular_velocity_right = 0.0f;
        }
        if (LEFT_WHEEL_INDEX == 0) {
            actuator_update_setpoint(&robot_platform.actuator1,
                                     setpoint.angular_velocity_left);
            actuator_update_setpoint(&robot_platform.actuator2,
                                     setpoint.angular_velocity_right);
        } else {
            actuator_update_setpoint(&robot_platform.actuator1,
                                     setpoint.angular_velocity_right);
            actuator_update_setpoint(&robot_platform.actuator2,
                                     setpoint.angular_velocity_left);
        }
    }
}

static void state_estimation_task(void *argument) {
    osal_loop_timer_t timer;
    osal_loop_timer_init(&timer, STATE_ESTIMATION_PERIOD_MS);
    for (;;) {
        float dt = osal_loop_timer_wait(&timer);

        actuator_state_sample_t state1;
        actuator_state_sample_t state2;
        if (osal_queue_peek(robot_platform.actuator1.state_queue_handle,
                            &state1, 0) != 0) {
            state1.angular_velocity = 0.0f;
        }
        if (osal_queue_peek(robot_platform.actuator2.state_queue_handle,
                            &state2, 0) != 0) {
            state2.angular_velocity = 0.0f;
        }

        float angular_velocity_left;
        float angular_velocity_right;
        if (LEFT_WHEEL_INDEX == 0) {
            angular_velocity_left = state1.angular_velocity;
            angular_velocity_right = state2.angular_velocity;
        } else {
            angular_velocity_left = state2.angular_velocity;
            angular_velocity_right = state1.angular_velocity;
        }

        robot_platform.estimator.predict(robot_platform.estimator.impl,
                                         angular_velocity_left,
                                         angular_velocity_right, dt);

        pose_estimator_state_t odometry_state;
        robot_platform.estimator.get_state(robot_platform.estimator.impl,
                                           &odometry_state);
        osal_queue_overwrite(robot_platform.odometry_mailbox, &odometry_state);
    }
}

void app_get_joint_state(joint_state_t *joint_state) {
    if (LEFT_WHEEL_INDEX == 0) {
        actuator_get_state(&robot_platform.actuator1,
                           &joint_state->angular_position[0],
                           &joint_state->angular_velocity[0]);
        actuator_get_state(&robot_platform.actuator2,
                           &joint_state->angular_position[1],
                           &joint_state->angular_velocity[1]);
    } else {
        actuator_get_state(&robot_platform.actuator1,
                           &joint_state->angular_position[1],
                           &joint_state->angular_velocity[1]);
        actuator_get_state(&robot_platform.actuator2,
                           &joint_state->angular_position[0],
                           &joint_state->angular_velocity[0]);
    }
}

void app_request_joint_space_setpoint(float angular_velocity_left,
                                      float angular_velocity_right) {
    wheel_vel_setpoint_t setpoint = {
        .angular_velocity_left = angular_velocity_left,
        .angular_velocity_right = angular_velocity_right,
    };
    osal_queue_overwrite(robot_platform.wheel_vel_setpoint_mailbox, &setpoint);
}

void app_get_odometry(pose_estimator_state_t *state) {
    osal_queue_peek(robot_platform.odometry_mailbox, state, 0);
}

void app_update_actuator_kp(float kp) {
    actuator_set_controller_kp(&robot_platform.actuator1, kp);
    actuator_set_controller_kp(&robot_platform.actuator2, kp);
}

void app_update_actuator_ki(float ki) {
    actuator_set_controller_ki(&robot_platform.actuator1, ki);
    actuator_set_controller_ki(&robot_platform.actuator2, ki);
}
