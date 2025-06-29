/**
 * @file micro_ros_transport.c
 * @brief Transport layer implementation for micro-ROS
 *
 * This file provides the implementation of the transport layer for micro-ROS.
 */

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int32.h>

#include "app.h"
#include "imu.h"
#include "microros_transport_config.h"
#include "osal_port.h"

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t encoder1_publisher;
std_msgs__msg__UInt32 encoder1_msg;

rcl_publisher_t encoder2_publisher;
std_msgs__msg__UInt32 encoder2_msg;

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
    imu_get_sample(&sample, OSAL_MAX_DELAY);

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

void transport_encoder_init(void *context) {
    transport_context_t *transport_context = (transport_context_t *)context;

    // publisher initialization
    rclc_publisher_init_default(
        &encoder1_publisher, transport_context->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "encoder1_position");
    encoder1_msg.data = 0;

    rclc_publisher_init_default(
        &encoder2_publisher, transport_context->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "encoder2_position");
    encoder2_msg.data = 0;
}

void transport_encoder_publish(void) {
    encoder1_msg.data = (uint32_t)app_get_left_encoder_value();
    encoder2_msg.data = (uint32_t)app_get_right_encoder_value();

    rcl_ret_t ret = rcl_publish(&encoder1_publisher, &encoder1_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing (line %d)\n", __LINE__);
    }

    ret = rcl_publish(&encoder2_publisher, &encoder2_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing (line %d)\n", __LINE__);
    }
}