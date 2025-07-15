/**
 * @file diff_drive.c
 * @brief Differential drive controller module
 */

#include "diff_drive.h"
#include <math.h>

void diff_drive_init(diff_drive_t *self, float wheel_diameter,
                     float wheel_distance) {
    self->wheel_diameter = wheel_diameter;
    self->wheel_distance = wheel_distance;
    self->theta = 0.0f;
    self->omega = 0.0f;
    self->x = 0.0f;
    self->y = 0.0f;
    self->vx = 0.0f;
    self->vy = 0.0f;
}

void diff_drive_map_wheel_to_platform(diff_drive_t *self,
                                      float angular_velocity_left,
                                      float angular_velocity_right,
                                      float *linear_velocity,
                                      float *angular_velocity) {
    *linear_velocity = (angular_velocity_left + angular_velocity_right) *
                       self->wheel_diameter / 4;
    *angular_velocity = (angular_velocity_right - angular_velocity_left) *
                        (self->wheel_diameter / 2) / self->wheel_distance;
}

void diff_drive_map_platform_to_wheel(diff_drive_t *self, float linear_velocity,
                                      float angular_velocity,
                                      float *angular_velocity_left,
                                      float *angular_velocity_right) {
    *angular_velocity_left =
        (linear_velocity - angular_velocity * self->wheel_distance / 2) /
        (self->wheel_diameter / 2);
    *angular_velocity_right =
        (linear_velocity + angular_velocity * self->wheel_distance / 2) /
        (self->wheel_distance / 2);
}

void diff_drive_update_kinematics(diff_drive_t *self, float linear_velocity,
                                  float angular_velocity, float delta_time) {
    self->theta += angular_velocity * delta_time;
    self->omega = angular_velocity;
    self->vx = linear_velocity * cosf(self->theta);
    self->vy = linear_velocity * sinf(self->theta);
    self->x += self->vx * delta_time;
    self->y += self->vy * delta_time;
}
