/**
 * @file diff_drive_estimator.c
 * @brief Differential drive adapter for the pose estimator interface.
 */

#include "diff_drive_estimator.h"

#include <string.h>

static void predict(void *self, float angular_velocity_left,
                    float angular_velocity_right, float dt) {
    diff_drive_t *diff_drive = (diff_drive_t *)self;
    float linear_velocity;
    float angular_velocity;
    diff_drive_map_wheels_to_body(diff_drive, angular_velocity_left,
                                  angular_velocity_right, &linear_velocity,
                                  &angular_velocity);
    diff_drive_update_kinematics(diff_drive, linear_velocity, angular_velocity,
                                 dt);
}

static void correct(void *self, float yaw_rate) {
    (void)self;
    (void)yaw_rate;
}

static void get_state(const void *self, pose_estimator_state_t *state) {
    const diff_drive_t *diff_drive = (const diff_drive_t *)self;
    state->x = diff_drive->x;
    state->y = diff_drive->y;
    state->theta = diff_drive->theta;
    state->vx = diff_drive->vx;
    state->omega = diff_drive->omega;
    memset(state->covariance, 0, sizeof(state->covariance));
}

static void set_state(void *self, const pose_estimator_state_t *state) {
    diff_drive_t *diff_drive = (diff_drive_t *)self;
    diff_drive->x = state->x;
    diff_drive->y = state->y;
    diff_drive->theta = state->theta;
}

void diff_drive_estimator_init(pose_estimator_t *estimator,
                               diff_drive_t *diff_drive) {
    estimator->predict = predict;
    estimator->correct = correct;
    estimator->get_state = get_state;
    estimator->set_state = set_state;
    estimator->impl = diff_drive;
}
