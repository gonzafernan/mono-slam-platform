/**
 * @file micro_ros_transport.c
 * @brief Transport layer implementation for micro-ROS
 *
 * This file provides the implementation of the transport layer for micro-ROS.
 */

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

#include "microros_transport_config.h"
#include "imu.h"

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t mag_x, mag_y, mag_z;

double accel_x_d = 0.0, accel_y_d = 0.0, accel_z_d = 0.0;
double gyro_x_d = 0.0, gyro_y_d = 0.0, gyro_z_d = 0.0;

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
    rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing (line %d)\n", __LINE__);
    }

    if (imu_read_accelerometer(&accel_x, &accel_y, &accel_z)) {
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;
    } else {
        accel_x_d = (double)accel_x / 16384.0;
        accel_y_d = (double)accel_y / 16384.0;
        accel_z_d = (double)accel_z / 16384.0;
        imu_msg.linear_acceleration.x = accel_x_d;
        imu_msg.linear_acceleration.y = accel_y_d;
        imu_msg.linear_acceleration.z = accel_z_d;
    }

    if (imu_read_gyroscope(&gyro_x, &gyro_y, &gyro_z)) {
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
    } else {
        imu_msg.angular_velocity.x = (double)gyro_x / 131.0;
        imu_msg.angular_velocity.y = (double)gyro_y / 131.0;
        imu_msg.angular_velocity.z = (double)gyro_z / 131.0;
    }

    if (imu_read_magnetometer(&mag_x, &mag_y, &mag_z)) {
        imu_msg.linear_acceleration_covariance[0] = 0.0;
        imu_msg.linear_acceleration_covariance[1] = 0.0;
        imu_msg.linear_acceleration_covariance[3] = 0.0;
    } else {
        imu_msg.linear_acceleration_covariance[0] = (double)mag_x;
        imu_msg.linear_acceleration_covariance[1] = (double)mag_y;
        imu_msg.linear_acceleration_covariance[2] = (double)mag_z;
    }
    // imu_msg.orientation.x = get_roll_from_accel(accel_x_d, accel_y_d, accel_z_d);
    // imu_msg.orientation.y = get_pitch_from_accel(accel_x_d, accel_y_d, accel_z_d);
}
