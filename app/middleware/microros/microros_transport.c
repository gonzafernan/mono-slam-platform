/**
 * @file micro_ros_transport.c
 * @brief Transport layer implementation for micro-ROS
 *
 * This file provides the implementation of the transport layer for micro-ROS.
 */

#include <stdio.h>
#include <string.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdio.h>

#include "app.h"
#include "app_config.h"
#include "imu.h"
#include "osal_port.h"
#include "rcl/types.h"
#include "std_msgs/msg/detail/float32_multi_array__functions.h"
#include "std_msgs/msg/detail/float32_multi_array__struct.h"
#include "transport.h"

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void *state);

static void transport_task(void *argument);
static void publisher_timer_callback(rcl_timer_t *timer,
                                     int64_t last_call_time);

/**
 * @brief Micro-ROS core components.
 */
static struct {
    void *context;
    void *task_handle;
    rcl_node_t node;
    rclc_executor_t executor;
} microros_transport;

/**
 * @brief Micro-ROS application components.
 */
static struct {
    rcl_timer_t joint_state_publisher_timer; /*!> Joint state publisher
                                                micro-ROS timer. */
    rcl_timer_t
        imu_publisher_timer; /*!> IMU measurement publisher micro-ROS timer. */
    rcl_publisher_t imu_publisher; /*!> IMU micro-ROS publisher. */
    sensor_msgs__msg__Imu imu_msg; /*!> IMU micro-ROS publisher message type. */

    rcl_publisher_t
        joint_state_publisher; /*!> Joint state micro-ROS publisher. */
    sensor_msgs__msg__JointState
        joint_state_msg; /*!> Joint state micro-ROS publisher message type. */
    char name_buffer[ACTUATED_JOINTS_NUMBER]
                    [MAX_JOINT_NAME_LENGTH]; /*!> Buffer for joint state
                               value name. */
    rosidl_runtime_c__String name_array[ACTUATED_JOINTS_NUMBER]; /*!> String for
                                                       joint state value. */
    double position_array[ACTUATED_JOINTS_NUMBER]; /*!> Buffer for joint state
                                                      position. */
    double velocity_array[ACTUATED_JOINTS_NUMBER]; /*!> Buffer for joint state
                                                      velocity. */

    rcl_subscription_t cmd_joint_space_subscriber; /*!> Joint space command
                                                      micro-ROS subscription. */
    std_msgs__msg__Float32MultiArray cmd_joint_space_msg;
    float cmd_joint_space_array[ACTUATED_JOINTS_NUMBER];

    rclc_parameter_server_t
        parameter_server; /*!> micro-ROS application parameter server. */
} microros_transport_components;

int transport_init(void *transport_context, void *task_attibutes) {
    microros_transport.context = transport_context;
    // micro-ROS transport task initialization
    microros_transport.task_handle =
        osal_task_static_create(transport_task, NULL, task_attibutes);
    return 0;
}

static void transport_task(void *argument) {
    // micro-ROS configuration
    rmw_uros_set_custom_transport(
        true, microros_transport.context, cubemx_transport_open,
        cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator =
        rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
        // return -1;
    }

    rclc_support_t support;
    rcl_allocator_t allocator;

    allocator = rcl_get_default_allocator();
    // create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&microros_transport.node, "vizcc_mcu", "", &support);
    // executor
    microros_transport.executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&microros_transport.executor, &support.context,
                       RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 7, &allocator);
    unsigned int rcl_executor_timeout = 10;
    rclc_executor_set_timeout(&microros_transport.executor,
                              RCL_MS_TO_NS(rcl_executor_timeout));

    // micro-ROS transport components initialization
    unsigned int rcl_timer_timeout = 10;
    rclc_timer_init_default(
        &microros_transport_components.joint_state_publisher_timer, &support,
        RCL_MS_TO_NS(rcl_timer_timeout), publisher_timer_callback);
    rclc_executor_add_timer(
        &microros_transport.executor,
        &microros_transport_components.joint_state_publisher_timer);

    rcl_timer_timeout = 20;
    rclc_timer_init_default(&microros_transport_components.imu_publisher_timer,
                            &support, RCL_MS_TO_NS(rcl_timer_timeout),
                            publisher_timer_callback);
    rclc_executor_add_timer(&microros_transport.executor,
                            &microros_transport_components.imu_publisher_timer);
    if (transport_imu_init() < 0) {
        printf("Error on IMU transport init (line %d).\r\n", __LINE__);
    }
    if (transport_joint_state_init() < 0) {
        printf("Error on joint state transport init (line %d).\r\n", __LINE__);
    }
    if (transport_command_joint_space_init() < 0) {
        printf("Error on joint state transport init (line %d).\r\n", __LINE__);
    }
    rclc_executor_prepare(&microros_transport.executor);
    rclc_executor_spin(&microros_transport.executor);
}

static void cmd_joint_space_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    float angular_velocity_left = msg->data.data[0];
    float angular_velocity_right = msg->data.data[1];
    // TODO: This should via a queue provided in
    // transport_command_joint_space_init
    app_update_joint_space_setpoint(angular_velocity_left,
                                    angular_velocity_right);
}

int transport_command_joint_space_init(void) {
    rcl_ret_t rc;

    std_msgs__msg__Float32MultiArray__init(
        &microros_transport_components.cmd_joint_space_msg);
    microros_transport_components.cmd_joint_space_msg.data.data =
        microros_transport_components.cmd_joint_space_array;
    microros_transport_components.cmd_joint_space_msg.data.capacity =
        ACTUATED_JOINTS_NUMBER;
    microros_transport_components.cmd_joint_space_msg.data.size = 0;

    rc = rclc_subscription_init_best_effort(
        &microros_transport_components.cmd_joint_space_subscriber,
        &microros_transport.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/vizcc_mcu/wheel_vel_cmd");
    if (rc != RCL_RET_OK) {
        printf("Error on joint space command subscription init (line %d).\r\n",
               __LINE__);
        return -1;
    }
    rc = rclc_executor_add_subscription(
        &microros_transport.executor,
        &microros_transport_components.cmd_joint_space_subscriber,
        &microros_transport_components.cmd_joint_space_msg,
        &cmd_joint_space_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        printf(
            "Error on addition of joint space command subscription to executor "
            "(line %d).\r\n",
            __LINE__);
        return -1;
    }
    return 0;
}

int transport_imu_init(void) {
    rcl_ret_t rc;

    // publisher initialization
    rc = rclc_publisher_init_default(
        &microros_transport_components.imu_publisher, &microros_transport.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_publisher");
    if (rc != RCL_RET_OK) {
        printf("Error on IMU publisher init (line %d).\r\n", __LINE__);
        return -1;
    }

    // message initialization
    microros_transport_components.imu_msg.header.stamp.sec = 0;
    microros_transport_components.imu_msg.header.stamp.nanosec = 0;
    microros_transport_components.imu_msg.header.frame_id.data = "imu_frame";
    microros_transport_components.imu_msg.header.frame_id.size =
        strlen(microros_transport_components.imu_msg.header.frame_id.data);
    microros_transport_components.imu_msg.header.frame_id.capacity =
        microros_transport_components.imu_msg.header.frame_id.size + 1;
    microros_transport_components.imu_msg.orientation.x = 0.0;
    microros_transport_components.imu_msg.orientation.y = 0.0;
    microros_transport_components.imu_msg.orientation.z = 0.0;
    microros_transport_components.imu_msg.orientation.w = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[0] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[1] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[2] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[3] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[4] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[5] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[6] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[7] = 0.0;
    microros_transport_components.imu_msg.orientation_covariance[8] = 0.0;
    microros_transport_components.imu_msg.angular_velocity.x = 0.0;
    microros_transport_components.imu_msg.angular_velocity.y = 0.0;
    microros_transport_components.imu_msg.angular_velocity.z = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[0] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[1] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[2] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[3] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[4] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[5] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[6] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[7] = 0.0;
    microros_transport_components.imu_msg.angular_velocity_covariance[8] = 0.0;
    microros_transport_components.imu_msg.linear_acceleration.x = 0.0;
    microros_transport_components.imu_msg.linear_acceleration.y = 0.0;
    microros_transport_components.imu_msg.linear_acceleration.z = 0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[0] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[1] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[2] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[3] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[4] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[5] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[6] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[7] =
        0.0;
    microros_transport_components.imu_msg.linear_acceleration_covariance[8] =
        0.0;
    return 0;
}

void transport_imu_publish(void) {
    imu_sample_t sample;
    // TODO: This should be from a queue provided in transport_imu_init
    imu_get_sample(&sample, 0);

    microros_transport_components.imu_msg.linear_acceleration.x =
        sample.accel_x;
    microros_transport_components.imu_msg.linear_acceleration.y =
        sample.accel_y;
    microros_transport_components.imu_msg.linear_acceleration.z =
        sample.accel_z;
    microros_transport_components.imu_msg.angular_velocity.x = sample.gyro_x;
    microros_transport_components.imu_msg.angular_velocity.y = sample.gyro_y;
    microros_transport_components.imu_msg.angular_velocity.z = sample.gyro_z;
    microros_transport_components.imu_msg.linear_acceleration_covariance[0] =
        sample.mag_x;
    microros_transport_components.imu_msg.linear_acceleration_covariance[1] =
        sample.mag_y;
    microros_transport_components.imu_msg.linear_acceleration_covariance[2] =
        sample.mag_z;

    rcl_ret_t rc = rcl_publish(&microros_transport_components.imu_publisher,
                               &microros_transport_components.imu_msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("Error IMU publishing (line %d)\n", __LINE__);
    }
}

int transport_joint_state_init(void) {
    rcl_ret_t rc;

    // publisher initialization
    rc = rclc_publisher_init_default(
        &microros_transport_components.joint_state_publisher,
        &microros_transport.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/vizcc_mcu/joint_states");
    if (rc != RCL_RET_OK) {
        printf("Error on joint state publisher init (line %d).\r\n", __LINE__);
        return -1;
    }

    // message initialization
    sensor_msgs__msg__JointState__init(
        &microros_transport_components.joint_state_msg);
    microros_transport_components.joint_state_msg.name.data =
        microros_transport_components.name_array;
    microros_transport_components.joint_state_msg.name.size =
        ACTUATED_JOINTS_NUMBER;
    microros_transport_components.joint_state_msg.name.capacity =
        ACTUATED_JOINTS_NUMBER;

    for (int i = 0; i < ACTUATED_JOINTS_NUMBER; i++) {
        microros_transport_components.name_array[i].data =
            microros_transport_components.name_buffer[i];
        microros_transport_components.name_array[i].size =
            0;  // No names provided
        microros_transport_components.name_array[i].capacity =
            MAX_JOINT_NAME_LENGTH;
    }

    microros_transport_components.joint_state_msg.position.data =
        microros_transport_components.position_array;
    microros_transport_components.joint_state_msg.position.size =
        ACTUATED_JOINTS_NUMBER;
    microros_transport_components.joint_state_msg.position.capacity =
        ACTUATED_JOINTS_NUMBER;

    microros_transport_components.joint_state_msg.velocity.data =
        microros_transport_components.velocity_array;
    microros_transport_components.joint_state_msg.velocity.size =
        ACTUATED_JOINTS_NUMBER;
    microros_transport_components.joint_state_msg.velocity.capacity =
        ACTUATED_JOINTS_NUMBER;

    microros_transport_components.joint_state_msg.effort.data = NULL;
    microros_transport_components.joint_state_msg.effort.size = 0;
    microros_transport_components.joint_state_msg.effort.capacity = 0;

    snprintf(microros_transport_components.name_buffer[0],
             MAX_JOINT_NAME_LENGTH, "left_wheel");
    microros_transport_components.name_array[0].size =
        strlen(microros_transport_components.name_buffer[0]);

    snprintf(microros_transport_components.name_buffer[1],
             MAX_JOINT_NAME_LENGTH, "right_wheel");
    microros_transport_components.name_array[1].size =
        strlen(microros_transport_components.name_buffer[1]);
    return 0;
}

void transport_joint_state_publish(void) {
    joint_state_t joint_state;
    // TODO: This should be provided by a queue in transport_joint_state_init
    app_get_joint_state(&joint_state);

    microros_transport_components.joint_state_msg.position.data[0] =
        joint_state.angular_position[0];
    microros_transport_components.joint_state_msg.position.data[1] =
        joint_state.angular_position[1];
    microros_transport_components.joint_state_msg.velocity.data[0] =
        joint_state.angular_velocity[0];
    microros_transport_components.joint_state_msg.velocity.data[1] =
        joint_state.angular_velocity[1];

    rcl_ret_t rc =
        rcl_publish(&microros_transport_components.joint_state_publisher,
                    &microros_transport_components.joint_state_msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("Error joint state publishing (line %d)\n", __LINE__);
    }
}

void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == &microros_transport_components.joint_state_publisher_timer) {
        transport_joint_state_publish();
    } else if (timer == &microros_transport_components.imu_publisher_timer) {
        transport_imu_publish();
    } else {
        printf(
            "Error joint state publisher timer callback execution without "
            "handle (line %d)\r\n",
            __LINE__);
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
        printf("Creating new parameter %s\r\n", new_param->name.data);
    } else if (new_param == NULL) {
        printf("Deleting parameter %s\r\n", old_param->name.data);
    } else {
        printf("Parameter %s modified.", old_param->name.data);
        if (strcmp(new_param->name.data, "controller.kp") == 0) {
            printf("Updated controller kp: %.4f",
                   new_param->value.double_value);
            app_update_actuator_kp((float)new_param->value.double_value);
        }
        if (strcmp(new_param->name.data, "controller.ki") == 0) {
            printf("Updated controller ki: %.4f",
                   new_param->value.double_value);
            app_update_actuator_ki((float)new_param->value.double_value);
        }
    }
    return true;
}

int transport_parameter_server_init(void) {
    rcl_ret_t rc = rclc_parameter_server_init_default(
        &microros_transport_components.parameter_server,
        &microros_transport.node);
    if (rc != RCL_RET_OK) {
        printf("Error on parameter server init (line %d)\r\n", __LINE__);
        return -1;
    }
    rc = rclc_executor_add_parameter_server(
        &microros_transport.executor,
        &microros_transport_components.parameter_server,
        transport_on_parameter_modification_callback);
    if (rc != RCL_RET_OK) {
        printf("Error adding parameter server to executor (line %d)\r\n",
               __LINE__);
        return -1;
    }

    rc = rclc_add_parameter(&microros_transport_components.parameter_server,
                            "controller.kp", RCLC_PARAMETER_DOUBLE);
    if (rc != RCL_RET_OK) {
        printf("Error adding parameter kp to server (line %d)\r\n", __LINE__);
        return -1;
    }
    rc = rclc_add_parameter_description(
        &microros_transport_components.parameter_server, "controller.kp",
        "Controller kp gain", "");
    if (rc != RCL_RET_OK) {
        printf("Error setting description for parameter kp (line %d)\r\n",
               __LINE__);
        return -1;
    }

    rc = rclc_add_parameter(&microros_transport_components.parameter_server,
                            "controller.ki", RCLC_PARAMETER_DOUBLE);
    if (rc != RCL_RET_OK) {
        printf("Error adding parameter ki to server (line %d)\r\n", __LINE__);
        return -1;
    }
    rc = rclc_add_parameter_description(
        &microros_transport_components.parameter_server, "controller.ki",
        "Controller ki gain", "");
    if (rc != RCL_RET_OK) {
        printf("Error setting description for parameter ki (line %d)\r\n",
               __LINE__);
        return -1;
    }
    return 0;
}
