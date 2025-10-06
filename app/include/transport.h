/**
 * @file transport.h
 * @brief Transport layer header file
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the transport layer.
 * @param transport_context Pointer to the transport context.
 * @param task_attributes Pointer to the RTOS task attributes.
 * @returns -1 if fail 0 if success.
 */
int transport_init(void *transport_context, void *task_attributes);

/**
 * @brief Initialize the transport layer for joint space command data.
 * @returns -1 if fail 0 if success.
 */
int transport_command_joint_space_init(void);

/**
 * @brief Initialize the transport layer for velocity command data.
 * @param context Pointer to the transport context
 */
void transport_command_velocity_init(void *context);

/**
 * @brief Initialize the transport layer for IMU data.
 * @returns -1 if fail 0 if success.
 */
int transport_imu_init(void);

/**
 * @brief Publish IMU data through the transport layer.
 * This function is called to publish the IMU data through the transport layer.
 */
void transport_imu_publish(void);

/**
 * @brief Initialize the transport layer for joint state data.
 * This function initializes the transport layer for joint state data.
 * @returns -1 if fail 0 if success.
 */
int transport_joint_state_init(void);

/**
 * @brief Publish joint state data through the transport layer.
 * This function is called to publish the joint state data through the transport
 * layer.
 */
void transport_joint_state_publish(void);

/**
 * @brief Initialize the transport layer for parameter server.
 * @returns -1 if fail 0 if success.
 */
int transport_parameter_server_init(void);

#ifdef __cplusplus
}
#endif

#endif  // TRANSPORT_H
