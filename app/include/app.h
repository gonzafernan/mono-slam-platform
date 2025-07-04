/**
 * @file app.h
 * @brief Main application header file
 *
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "actuator.h"

typedef struct {
    double angular_position[ACTUATED_JOINTS_NUMBER];
    double angular_velocity[ACTUATED_JOINTS_NUMBER];
} joint_state_t;

/**
 * @brief Application initialization
 * @param imu Pointer to the IMU context
 * @param actuator1_args Pointer to the actuator 1 arguments
 * @param actuator2_args Pointer to the actuator 2 arguments
 * @return 0 on success, -1 on failure
 */
int app_init(void *imu, actuator_args_t *actuator1_args,
             actuator_args_t *actuator2_args);

/**
 * @brief Get the current joint state
 * @param joint_state Pointer to a joint_state_t structure to store the state
 */
void app_get_joint_state(joint_state_t *joint_state);

#ifdef __cplusplus
}
#endif

#endif  // APP_H