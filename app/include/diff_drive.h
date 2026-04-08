/**
 * @file diff_drive.h
 * @brief Header for differential drive controller module
 */

#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float wheel_diameter; /*!> Wheels diameter in milimeters. */
    float wheel_distance; /*!> Distance between wheels in milimeters. */
    float theta;          /*!> Cartesian angle in radians. */
    float omega; /*!> Cartesian angular velocity in radians per second. */
    float x;     /*!> Cartesian position in x axis in milimeters per second. */
    float y;     /*!> Cartesian position in y axis in milimeters per second. */
    float vx;    /*!> Cartesian velocity in x axis in milimeters. */
    float vy;    /*!> Cartesian velocity in y axis in milimeters. */
} diff_drive_t;

/**
 * @brief Differential drive controller initialization.
 * @param self Pointer to the differential drive controller.
 * @param wheel_diameter Wheels diameter in milimeters.
 * @param wheel_distance Distance between wheels in milimeters.
 * @return 0 on success, -1 on failure
 */
int diff_drive_init(diff_drive_t *self, float wheel_diameter,
                    float wheel_distance);

/**
 * @brief Map wheels angular velocity to linear and angular velocity.
 * @param self Pointer to the differential drive controller.
 * @param angular_velocity_left Left wheel angular velocity.
 * @param angular_velocity_right Right wheel angular velocity.
 * @param linear_velocity Pointer to mapped linear velocity.
 * @param angular_velocity Pointer to mapped angular velocity.
 */
void diff_drive_map_wheels_to_body(diff_drive_t *self,
                                   float angular_velocity_left,
                                   float angular_velocity_right,
                                   float *linear_velocity,
                                   float *angular_velocity);

/**
 * @brief Map linear and angular velocity to wheels angular velocity.
 * @param self Pointer to the differential drive controller.
 * @param linear_velocity Linear velocity in milimeters per second.
 * @param angular_velocity Angular velocity in radians per second.
 * @param angular_velocity_left Pointer to mapped left wheel angular velocity.
 * @param angular_velocity_right Pointer to mapped right wheel angular velocity.
 */
void diff_drive_map_body_to_wheels(diff_drive_t *self, float linear_velocity,
                                   float angular_velocity,
                                   float *angular_velocity_left,
                                   float *angular_velocity_right);

/**
 * @brief Update controller with new angular and linear velocity.
 * Update controller kinematics.
 * @param self Pointer to the differential drive controller.
 * @param linear_velocity Linear velocity in milimeters per second.
 * @param angular_velocity Angular velocity in radians per second.
 * @param delta_time Time elapsed since last update in seconds.
 */
void diff_drive_update_kinematics(diff_drive_t *self, float linear_velocity,
                                  float angular_velocity, float delta_time);

#ifdef __cplusplus
}
#endif

#endif  // DIFF_FRIVE_H
