/**
 * @file micro_ros_transport.c
 * @brief Transport layer implementation for micro-ROS
 *
 * This file provides the implementation of the transport layer for micro-ROS.
 */

#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>

#include "app.h"
#include "app_config.h"
#include "imu.h"
#include "microros_transport_config.h"
#include "osal_port.h"
#include "std_msgs/msg/detail/float32_multi_array__functions.h"
#include "std_msgs/msg/detail/float32_multi_array__struct.h"

rclc_parameter_server_t parameter_server;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_state_msg;
char name_buffer[ACTUATED_JOINTS_NUMBER][MAX_JOINT_NAME_LENGTH];
rosidl_runtime_c__String name_array[ACTUATED_JOINTS_NUMBER];
double position_array[ACTUATED_JOINTS_NUMBER];
double velocity_array[ACTUATED_JOINTS_NUMBER];

rcl_subscription_t cmd_joint_space_subscriber;
std_msgs__msg__Float32MultiArray cmd_joint_space_msg;
float cmd_joint_space_array[ACTUATED_JOINTS_NUMBER];

// geometry_msgs__msg__Twist cmd_vel_msg;

static void cmd_joint_space_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    float angular_velocity_left = msg->data.data[0];
    float angular_velocity_right = msg->data.data[1];
    // app_update_joint_space_setpoint(angular_velocity_left,
    //                                 angular_velocity_right);
    // printf("JS %.4f - %.4f\r\n", angular_velocity_left,
    // angular_velocity_right);
}

// void cmd_vel_callback(const void *msgin) {
//     const geometry_msgs__msg__Twist *msg =
//         (const geometry_msgs__msg__Twist *)msgin;
//     app_update_setpoint(msg->linear.x, msg->angular.z);
// }

int transport_command_joint_space_init(void *context) {
    transport_context_t *transport_context = (transport_context_t *)context;
    rcl_ret_t rc;

    std_msgs__msg__Float32MultiArray__init(&cmd_joint_space_msg);
    cmd_joint_space_msg.data.data = cmd_joint_space_array;
    cmd_joint_space_msg.data.capacity = ACTUATED_JOINTS_NUMBER;
    cmd_joint_space_msg.data.size = 0;

    rc = rclc_subscription_init_default(
        &cmd_joint_space_subscriber, transport_context->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/vizcc_mcu/wheel_vel_cmd");
    if (rc != RCL_RET_OK) {
        printf("ERROR: Unable to create joint space command subscription.\r\n");
        return -1;
    }
    rc = rclc_executor_add_subscription(
        transport_context->executor, &cmd_joint_space_subscriber,
        &cmd_joint_space_msg, &cmd_joint_space_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        printf(
            "ERROR: Unable to add joint space command subscription to "
            "executor.\r\n");
        return -1;
    }
    return 0;
}

// void transport_command_velocity_init(void *context) {
//     transport_context_t *transport_context = (transport_context_t *)context;
//     rclc_subscription_init_default(
//         &cmd_vel_subscriber, transport_context->node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
//         "/diff_drive_controller/cmd_vel");
//     rclc_executor_add_subscription(transport_context->executor,
//                                    &cmd_vel_subscriber, &cmd_vel_msg,
//                                    &cmd_vel_callback, ON_NEW_DATA);
// }

void transport_imu_init(void *context) {
    transport_context_t *transport_context = (transport_context_t *)context;

    // publisher initialization
    rclc_publisher_init_default(
        &imu_publisher, transport_context->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_publisher");

    // message initialization
    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nanosec = 0;
    imu_msg.header.frame_id.data = "imu_frame";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;
    imu_msg.orientation_covariance[0] = 0.0;
    imu_msg.orientation_covariance[1] = 0.0;
    imu_msg.orientation_covariance[2] = 0.0;
    imu_msg.orientation_covariance[3] = 0.0;
    imu_msg.orientation_covariance[4] = 0.0;
    imu_msg.orientation_covariance[5] = 0.0;
    imu_msg.orientation_covariance[6] = 0.0;
    imu_msg.orientation_covariance[7] = 0.0;
    imu_msg.orientation_covariance[8] = 0.0;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.angular_velocity_covariance[0] = 0.0;
    imu_msg.angular_velocity_covariance[1] = 0.0;
    imu_msg.angular_velocity_covariance[2] = 0.0;
    imu_msg.angular_velocity_covariance[3] = 0.0;
    imu_msg.angular_velocity_covariance[4] = 0.0;
    imu_msg.angular_velocity_covariance[5] = 0.0;
    imu_msg.angular_velocity_covariance[6] = 0.0;
    imu_msg.angular_velocity_covariance[7] = 0.0;
    imu_msg.angular_velocity_covariance[8] = 0.0;
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    imu_msg.linear_acceleration_covariance[0] = 0.0;
    imu_msg.linear_acceleration_covariance[1] = 0.0;
    imu_msg.linear_acceleration_covariance[2] = 0.0;
    imu_msg.linear_acceleration_covariance[3] = 0.0;
    imu_msg.linear_acceleration_covariance[4] = 0.0;
    imu_msg.linear_acceleration_covariance[5] = 0.0;
    imu_msg.linear_acceleration_covariance[6] = 0.0;
    imu_msg.linear_acceleration_covariance[7] = 0.0;
    imu_msg.linear_acceleration_covariance[8] = 0.0;
}

void transport_imu_publish(void) {
    imu_sample_t sample;
    imu_get_sample(&sample, 0);

    imu_msg.linear_acceleration.x = sample.accel_x;
    imu_msg.linear_acceleration.y = sample.accel_y;
    imu_msg.linear_acceleration.z = sample.accel_z;
    imu_msg.angular_velocity.x = sample.gyro_x;
    imu_msg.angular_velocity.y = sample.gyro_y;
    imu_msg.angular_velocity.z = sample.gyro_z;
    imu_msg.linear_acceleration_covariance[0] = sample.mag_x;
    imu_msg.linear_acceleration_covariance[1] = sample.mag_y;
    imu_msg.linear_acceleration_covariance[2] = sample.mag_z;

    rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing (line %d)\n", __LINE__);
    }
}

void transport_joint_state_init(void *context) {
    transport_context_t *transport_context = (transport_context_t *)context;

    // publisher initialization
    rclc_publisher_init_default(
        &joint_state_publisher, transport_context->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/vizcc_mcu/joint_states");

    // message initialization
    sensor_msgs__msg__JointState__init(&joint_state_msg);
    joint_state_msg.name.data = name_array;
    joint_state_msg.name.size = ACTUATED_JOINTS_NUMBER;
    joint_state_msg.name.capacity = ACTUATED_JOINTS_NUMBER;

    for (int i = 0; i < ACTUATED_JOINTS_NUMBER; i++) {
        name_array[i].data = name_buffer[i];
        name_array[i].size = 0;  // No names provided
        name_array[i].capacity = MAX_JOINT_NAME_LENGTH;
    }

    joint_state_msg.position.data = position_array;
    joint_state_msg.position.size = ACTUATED_JOINTS_NUMBER;
    joint_state_msg.position.capacity = ACTUATED_JOINTS_NUMBER;

    joint_state_msg.velocity.data = velocity_array;
    joint_state_msg.velocity.size = ACTUATED_JOINTS_NUMBER;
    joint_state_msg.velocity.capacity = ACTUATED_JOINTS_NUMBER;

    joint_state_msg.effort.data = NULL;
    joint_state_msg.effort.size = 0;
    joint_state_msg.effort.capacity = 0;

    snprintf(name_buffer[0], MAX_JOINT_NAME_LENGTH, "left_wheel");
    name_array[0].size = strlen(name_buffer[0]);

    snprintf(name_buffer[1], MAX_JOINT_NAME_LENGTH, "right_wheel");
    name_array[1].size = strlen(name_buffer[1]);
}

void transport_joint_state_publish(void) {
    joint_state_t joint_state;
    app_get_joint_state(&joint_state);

    joint_state_msg.position.data[0] = joint_state.angular_position[0];
    joint_state_msg.position.data[1] = joint_state.angular_position[1];
    joint_state_msg.velocity.data[0] = joint_state.angular_velocity[0];
    joint_state_msg.velocity.data[1] = joint_state.angular_velocity[1];

    rcl_ret_t ret = rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing (line %d)\n", __LINE__);
    }
}

bool transport_on_parameter_modification_callback(const Parameter *old_param,
                                                  const Parameter *new_param,
                                                  void *context) {
    if (old_param == NULL && new_param == NULL) {
        printf("Callback error, both parameters are NULL\r\n");
        return false;
    }

    if (old_param == NULL) {
        printf("Creat(ing new parameter %s\r\n", new_param->name.data);
    } else if (new_param == NULL) {
        printf("Deleting parameter %s\r\n", old_param->name.data);
    } else {
        printf("Parameter %s modified.", old_param->name.data);
        if (strcmp(new_param->name.data, "controller/kp") == 0) {
            printf("Updated controller kp: %.4f",
                   new_param->value.double_value);
            app_update_actuator_kp((float)new_param->value.double_value);
        }
        if (strcmp(new_param->name.data, "controller/ki") == 0) {
            printf("Updated controller ki: %.4f",
                   new_param->value.double_value);
            app_update_actuator_ki((float)new_param->value.double_value);
        }
    }
    return true;
}

void transport_parameter_server_init(void *context) {
    transport_context_t *transport_context = (transport_context_t *)context;
    rcl_ret_t ret = rclc_parameter_server_init_default(&parameter_server,
                                                       transport_context->node);
    if (ret != RCL_RET_OK) {
        printf("Error on parameter server init (line %d)\r\n", __LINE__);
    }
    ret = rclc_executor_add_parameter_server(
        transport_context->executor, &parameter_server,
        transport_on_parameter_modification_callback);
    if (ret != RCL_RET_OK) {
        printf("Error adding parameter server to executor (line %d)\r\n",
               __LINE__);
    }

    ret = rclc_add_parameter(&parameter_server, "controller/kp",
                             RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) {
        printf("Error adding parameter kp to server (line %d)\r\n", __LINE__);
    }
    ret = rclc_add_parameter_description(&parameter_server, "controller/kp",
                                         "Controller kp gain", "");
    if (ret != RCL_RET_OK) {
        printf("Error setting description for parameter kp (line %d)\r\n",
               __LINE__);
    }

    ret = rclc_add_parameter(&parameter_server, "controller/ki",
                             RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) {
        printf("Error adding parameter ki to server (line %d)\r\n", __LINE__);
    }
    ret = rclc_add_parameter_description(&parameter_server, "controller/ki",
                                         "Controller ki gain", "");
    if (ret != RCL_RET_OK) {
        printf("Error setting description for parameter ki (line %d)\r\n",
               __LINE__);
    }
}
