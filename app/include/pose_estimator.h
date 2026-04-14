/**
 * @file pose_estimator.h
 * @brief 2D pose estimator interface for differential drive robots.
 */

#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pose estimator output state.
 *
 * Positions in millimeters, velocities in millimeters per second and
 * radians per second. Heading normalised to [-pi, pi].
 * Covariance is zero for implementations that do not model uncertainty.
 */
typedef struct {
    float x;                /*!< Position along x axis in millimeters. */
    float y;                /*!< Position along y axis in millimeters. */
    float theta;            /*!< Heading in radians. */
    float vx;               /*!< Linear velocity in millimeters per second. */
    float omega;            /*!< Angular velocity in radians per second. */
    float covariance[3][3]; /*!< State covariance [x, y, theta]. Zero if not
                               modelled. */
} pose_estimator_state_t;

/**
 * @brief Pose estimator interface.
 *
 * Function pointers to a concrete estimation strategy. All pointers must
 * be non-NULL after initialisation.
 */
typedef struct {
    /**
     * @brief Propagate state forward using wheel angular velocities.
     * @param self                   Pointer to the implementation context.
     * @param angular_velocity_left  Left wheel angular velocity.
     * @param angular_velocity_right Right wheel angular velocity.
     * @param dt                     Elapsed time since last call.
     */
    void (*predict)(void *self, float angular_velocity_left,
                    float angular_velocity_right, float dt);

    /**
     * @brief Correct state using a yaw rate measurement.
     * No-op for implementations that do not support measurement correction.
     * @param self      Pointer to the implementation context.
     * @param yaw_rate  Measured angular velocity around the vertical axis.
     */
    void (*correct)(void *self, float yaw_rate);

    /**
     * @brief Read the current estimated state.
     * @param self  Pointer to the implementation context.
     * @param state Output: current pose estimate.
     */
    void (*get_state)(const void *self, pose_estimator_state_t *state);

    /**
     * @brief Seed the estimator from an existing state.
     * @param self  Pointer to the implementation context.
     * @param state Input: pose to initialise from.
     */
    void (*set_state)(void *self, const pose_estimator_state_t *state);

    void *impl; /*!< Pointer to the implementation context. */
} pose_estimator_t;

#ifdef __cplusplus
}
#endif

#endif  // POSE_ESTIMATOR_H
