/**
 * @file test_diff_drive.cpp
 * @brief Unit tests for the differential drive kinematics module
 */

#include <gtest/gtest.h>

extern "C" {
#include "diff_drive.h"
}

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
