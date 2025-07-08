/**
 * @file test_pid.cpp
 * @brief Unit tests for the PID controller
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "pid.h"
}

TEST(PIDControllerTest, InitializationSetsAllFieldsToZero) {
    pid_controller_t pid;
    pid_init(&pid);

    EXPECT_FLOAT_EQ(pid.kp, 0.0f);
    EXPECT_FLOAT_EQ(pid.ki, 0.0f);
    EXPECT_FLOAT_EQ(pid.kd, 0.0f);
    EXPECT_FLOAT_EQ(pid.tau, 0.0f);
    EXPECT_FLOAT_EQ(pid.min_output, 0.0f);
    EXPECT_FLOAT_EQ(pid.max_output, 0.0f);
    EXPECT_FLOAT_EQ(pid.min_integral, 0.0f);
    EXPECT_FLOAT_EQ(pid.max_integral, 0.0f);
    EXPECT_FLOAT_EQ(pid.prev_input, 0.0f);
    EXPECT_FLOAT_EQ(pid.setpoint, 0.0f);
    EXPECT_FLOAT_EQ(pid.error_integral, 0.0f);
    EXPECT_FLOAT_EQ(pid.input_derivative, 0.0f);
}

TEST(PIDControllerTest, GainSettersUpdateValues) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_output_range(&pid, -1000.0f, 1000.0f);

    pid_set_kp(&pid, 2.5f);
    pid_set_ki(&pid, 0.5f);
    pid_set_kd(&pid, 0.1f);
    pid_set_tau(&pid, 0.05f);

    EXPECT_EQ(pid.min_output, -1000.0f);
    EXPECT_EQ(pid.max_output, 1000.0f);

    EXPECT_FLOAT_EQ(pid.kp, 2.5f);
    EXPECT_FLOAT_EQ(pid.ki, 0.5f);
    EXPECT_FLOAT_EQ(pid.kd, 0.1f);
    EXPECT_FLOAT_EQ(pid.tau, 0.05f);
}

TEST(PIDControllerTest, OutputIsSaturated) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_kp(&pid, 10.0f);
    pid_set_output_range(&pid, -5.0f, 5.0f);
    pid_set_setpoint(&pid, 1.0f);

    float output = pid_update(&pid, -10.0f, 0.1f);
    EXPECT_LE(output, 5.0f);
    EXPECT_GE(output, -5.0f);
}

TEST(PIDControllerTest, IntegralWindupIsClamped) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_ki(&pid, 10.0f);
    pid_set_integral_range(&pid, -1.0f, 1.0f);
    pid_set_setpoint(&pid, 1.0f);

    for (int i = 0; i < 100; ++i) {
        pid_update(&pid, 0.0f, 0.1f);
    }

    EXPECT_LE(pid.error_integral, 1.0f);
    EXPECT_GE(pid.error_integral, -1.0f);
}

TEST(PIDControllerTest, DerivativeRespondsToInputChange) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.0f);  // no low-pass filter
    pid_set_setpoint(&pid, 0.0f);

    pid_update(&pid, 0.0f, 0.1f);                  // initial input
    float output = pid_update(&pid, 10.0f, 0.1f);  // sudden jump

    EXPECT_LT(output, 0.0f);  // derivative acts against increasing input
}

TEST(PIDControllerTest, ZeroErrorGivesZeroOutput) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_kp(&pid, 1.0f);
    pid_set_ki(&pid, 1.0f);
    pid_set_kd(&pid, 1.0f);
    pid_set_setpoint(&pid, 10.0f);

    pid.prev_input = 10.0f;  // to cancel derivative action

    float output = pid_update(&pid, 10.0f, 0.1f);  // error = 0
    EXPECT_NEAR(output, 0.0f, 1e-5f);
}

TEST(PIDControllerTest, SetpointChangeImpactsOutput) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_kp(&pid, 1.0f);
    pid_set_setpoint(&pid, 0.0f);
    float out1 = pid_update(&pid, 0.0f, 0.1f);

    pid_set_setpoint(&pid, 5.0f);
    float out2 = pid_update(&pid, 0.0f, 0.1f);

    EXPECT_LT(out1, out2);
}

TEST(PIDControllerTest, HandlesZeroDeltaTime) {
    pid_controller_t pid;
    pid_init(&pid);

    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_kp(&pid, 1.0f);
    pid_set_kd(&pid, 1.0f);

    float output = pid_update(&pid, 1.0f, 0.0f);  // no time passed

    EXPECT_TRUE(std::isfinite(output));  // No NaN or Inf
}
