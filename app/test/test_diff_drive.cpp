/**
 * @file test_diff_drive.cpp
 * @brief Unit tests for the differential drive kinematics module
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "diff_drive.h"
}

// Fixture to create differential drive
class DiffDriveTest : public ::testing::Test {
   protected:
    diff_drive_t diff_drive;
    const float DIAMETER = 65.0f;
    const float DISTANCE = 160.0f;

    void SetUp() override { diff_drive_init(&diff_drive, DIAMETER, DISTANCE); }
};

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

TEST(DiffDriveInitTest, StoresWheelParameters) {
    diff_drive_t diff_drive;
    EXPECT_EQ(diff_drive_init(&diff_drive, 65.0f, 160.0f), 0);

    EXPECT_FLOAT_EQ(diff_drive.wheel_diameter, 65.0f);
    EXPECT_FLOAT_EQ(diff_drive.wheel_distance, 160.0f);
}

TEST(DiffDriveInitTest, ZerosAllStateFields) {
    diff_drive_t diff_drive;
    diff_drive_init(&diff_drive, 65.0f, 160.0f);

    EXPECT_FLOAT_EQ(diff_drive.theta, 0.0f);
    EXPECT_FLOAT_EQ(diff_drive.omega, 0.0f);
    EXPECT_FLOAT_EQ(diff_drive.x, 0.0f);
    EXPECT_FLOAT_EQ(diff_drive.y, 0.0f);
    EXPECT_FLOAT_EQ(diff_drive.vx, 0.0f);
    EXPECT_FLOAT_EQ(diff_drive.vy, 0.0f);
}

TEST(DiffDriveInitTest, ZeroDiameterReturnsError) {
    diff_drive_t diff_drive;
    EXPECT_EQ(diff_drive_init(&diff_drive, 0.0f, 160.0f), -1);
}

TEST(DiffDriveInitTest, ZeroDistanceReturnsError) {
    diff_drive_t diff_drive;
    EXPECT_EQ(diff_drive_init(&diff_drive, 65.0f, 0.0f), -1);
}

TEST(DiffDriveInitTest, NegativeDiameterReturnsError) {
    diff_drive_t diff_drive;
    EXPECT_EQ(diff_drive_init(&diff_drive, -1.0f, 160.0f), -1);
}

TEST(DiffDriveInitTest, NegativeDistanceReturnsError) {
    diff_drive_t diff_drive;
    EXPECT_EQ(diff_drive_init(&diff_drive, 65.0f, -1.0f), -1);
}

// ----------------------------------------------------------------------------
// Forward Kinematics
// ----------------------------------------------------------------------------

TEST_F(DiffDriveTest, ForwardKinematicsBothWheelsSameSpeed) {
    float linear_velocity, angular_velocity;
    diff_drive_map_wheels_to_body(&diff_drive, 10.0f, 10.0f, &linear_velocity,
                                  &angular_velocity);
    // v = (10+10) * 65/4 = 325 mm/s
    EXPECT_FLOAT_EQ(linear_velocity, 325.0f);
    EXPECT_FLOAT_EQ(angular_velocity, 0.0f);
}

TEST_F(DiffDriveTest, ForwardKinematicsOppositeWheelSpeeds) {
    float linear_velocity, angular_velocity;
    diff_drive_map_wheels_to_body(&diff_drive, 10.0f, -10.0f, &linear_velocity,
                                  &angular_velocity);
    // v = (10-10) * 65/4 = 0
    // omega = (-10-10) * 32.5 / 160 = -4.0625 rad/s
    EXPECT_FLOAT_EQ(linear_velocity, 0.0f);
    EXPECT_FLOAT_EQ(angular_velocity, -4.0625f);
}

TEST_F(DiffDriveTest, ForwardKinematicsSymmetricMotionNoRotation) {
    float linear_velocity, angular_velocity;
    diff_drive_map_wheels_to_body(&diff_drive, 5.0f, 5.0f, &linear_velocity,
                                  &angular_velocity);
    // v = (5+5) * 65/4 = 162.5 mm/s
    EXPECT_FLOAT_EQ(linear_velocity, 162.5f);
    EXPECT_FLOAT_EQ(angular_velocity, 0.0f);
}

TEST_F(DiffDriveTest, ForwardKinematicsRotationInPlace) {
    float linear_velocity, angular_velocity;
    diff_drive_map_wheels_to_body(&diff_drive, 5.0f, -5.0f, &linear_velocity,
                                  &angular_velocity);
    EXPECT_FLOAT_EQ(linear_velocity, 0.0f);
    EXPECT_NE(angular_velocity, 0.0f);
}

// ----------------------------------------------------------------------------
// Inverse Kinematics
// ----------------------------------------------------------------------------

TEST_F(DiffDriveTest, InverseKinematicsPureTranslationLeftWheel) {
    float omega_left, omega_right;
    diff_drive_map_body_to_wheels(&diff_drive, 162.5f, 0.0f, &omega_left,
                                  &omega_right);
    // omega_L = 162.5 / 32.5 = 5.0 rad/s
    EXPECT_FLOAT_EQ(omega_left, 5.0f);
}

TEST_F(DiffDriveTest, InverseKinematicsPureTranslationRightWheel) {
    float omega_left, omega_right;
    diff_drive_map_body_to_wheels(&diff_drive, 162.5f, 0.0f, &omega_left,
                                  &omega_right);
    // omega_R = 162.5 / 32.5 = 5.0 rad/s
    EXPECT_FLOAT_EQ(omega_right, 5.0f);
}

TEST_F(DiffDriveTest, InverseKinematicsRoundtrip) {
    const float omega_left_in = 5.0f;
    const float omega_right_in = 10.0f;

    float linear_velocity, angular_velocity;
    diff_drive_map_wheels_to_body(&diff_drive, omega_left_in, omega_right_in,
                                  &linear_velocity, &angular_velocity);

    float omega_left_out, omega_right_out;
    diff_drive_map_body_to_wheels(&diff_drive, linear_velocity,
                                  angular_velocity, &omega_left_out,
                                  &omega_right_out);

    EXPECT_NEAR(omega_left_out, omega_left_in, 1e-4f);
    EXPECT_NEAR(omega_right_out, omega_right_in, 1e-4f);
}

TEST_F(DiffDriveTest, InverseKinematicsRoundtripRotationInPlace) {
    const float omega_left_in = 5.0f;
    const float omega_right_in = -5.0f;

    float linear_velocity, angular_velocity;
    diff_drive_map_wheels_to_body(&diff_drive, omega_left_in, omega_right_in,
                                  &linear_velocity, &angular_velocity);

    float omega_left_out, omega_right_out;
    diff_drive_map_body_to_wheels(&diff_drive, linear_velocity,
                                  angular_velocity, &omega_left_out,
                                  &omega_right_out);

    EXPECT_NEAR(omega_left_out, omega_left_in, 1e-4f);
    EXPECT_NEAR(omega_right_out, omega_right_in, 1e-4f);
}
