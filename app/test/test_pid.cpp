/**
 * @file test_pid.cpp
 * @brief Unit tests for the PID controller
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

extern "C" {
#include "pid.h"
}

// Fixture: initialised controller with wide output and integral ranges.
// Individual tests set gains and setpoint as needed.
class PIDTest : public ::testing::Test {
   protected:
    pid_controller_t pid;

    void SetUp() override {
        pid_init(&pid);
        pid_set_output_range(&pid, -1000.0f, 1000.0f);
        pid_set_integral_range(&pid, -1000.0f, 1000.0f);
    }
};

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
    pid_set_setpoint(&pid, 5.0f);

    float output = pid_update(&pid, 1.0f, 0.0f);

    EXPECT_TRUE(std::isfinite(output));       // no NaN or Inf
    EXPECT_FLOAT_EQ(pid.error_integral, 0.0f); // integral must not accumulate
    EXPECT_FLOAT_EQ(pid.input_derivative, 0.0f); // derivative zeroed on zero denom
}

// ----------------------------------------------------------------------------
// Configuration — T-PID-CFG-005a
// ----------------------------------------------------------------------------

TEST_F(PIDTest, GainChangeTakesEffectImmediately) {
    pid_set_kp(&pid, 1.0f);
    pid_set_setpoint(&pid, 10.0f);

    pid_update(&pid, 7.0f, 0.1f);

    // Reset state manually so only P contributes on the next call
    pid.error_integral = 0.0f;
    pid.prev_input = 7.0f;

    pid_set_kp(&pid, 2.0f);
    float output = pid_update(&pid, 7.0f, 0.0f);  // dt=0: no integral, no derivative

    // error = 10 - 7 = 3, P = 2 * 3 = 6
    EXPECT_NEAR(output, 6.0f, 1e-4f);
}

// ----------------------------------------------------------------------------
// Proportional term — T-PID-P-001a, T-PID-P-001b, T-PID-P-003a
// ----------------------------------------------------------------------------

TEST_F(PIDTest, ProportionalTermComputedCorrectly) {
    pid_set_kp(&pid, 2.0f);
    pid_set_setpoint(&pid, 10.0f);

    float output = pid_update(&pid, 7.0f, 0.0f);  // dt=0: no integral, no derivative

    // error = 10 - 7 = 3; P = 2 * 3 = 6
    EXPECT_NEAR(output, 6.0f, 1e-4f);
}

TEST_F(PIDTest, ProportionalTermNegativeError) {
    pid_set_kp(&pid, 1.0f);
    pid_set_setpoint(&pid, 5.0f);

    float output = pid_update(&pid, 8.0f, 0.0f);  // dt=0: no integral, no derivative

    // error = 5 - 8 = -3; output = 1 * (-3) = -3
    EXPECT_NEAR(output, -3.0f, 1e-4f);
}

TEST_F(PIDTest, OutputScalesLinearlyWithKp) {
    pid_set_setpoint(&pid, 10.0f);

    pid_set_kp(&pid, 1.0f);
    float out1 = pid_update(&pid, 7.0f, 0.0f);

    pid_init(&pid);
    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_integral_range(&pid, -1000.0f, 1000.0f);
    pid_set_kp(&pid, 2.0f);
    pid_set_setpoint(&pid, 10.0f);
    float out2 = pid_update(&pid, 7.0f, 0.0f);

    EXPECT_NEAR(out2, 2.0f * out1, 1e-4f);
}

// ----------------------------------------------------------------------------
// Integral term — T-PID-I-001a/b, T-PID-I-003a, T-PID-I-004a, T-PID-I-005a
// ----------------------------------------------------------------------------

TEST_F(PIDTest, IntegralAccumulatesCorrectly) {
    pid_set_ki(&pid, 1.0f);
    pid_set_setpoint(&pid, 1.0f);

    for (int i = 0; i < 10; ++i) {
        pid_update(&pid, 0.0f, 0.1f);
    }

    // integral = 10 * 0.1 * 1.0 = 1.0
    EXPECT_NEAR(pid.error_integral, 1.0f, 1e-4f);
}

TEST_F(PIDTest, IntegralAccumulatesVaryingError) {
    pid_set_ki(&pid, 1.0f);

    pid_set_setpoint(&pid, 1.0f);
    pid_update(&pid, 0.0f, 0.1f);  // error=1, contribution=0.1

    pid_set_setpoint(&pid, 3.0f);
    pid_update(&pid, 0.0f, 0.1f);  // error=3, contribution=0.3

    // total integral = 0.1 + 0.3 = 0.4
    EXPECT_NEAR(pid.error_integral, 0.4f, 1e-4f);
}

TEST_F(PIDTest, OutputRampsLinearlyWithIntegral) {
    // kp=0, kd=0 (default), ki=1, setpoint=1, input=0 (constant)
    pid_set_ki(&pid, 1.0f);
    pid_set_setpoint(&pid, 1.0f);

    const float step = 0.1f;  // ki * error * dt = 1 * 1 * 0.1
    float prev_output = 0.0f;

    for (int i = 0; i < 5; ++i) {
        float output = pid_update(&pid, 0.0f, 0.1f);
        EXPECT_NEAR(output, prev_output + step, 1e-4f);
        prev_output = output;
    }
}

TEST_F(PIDTest, ZeroIntegralRangeKeepsIntegralAtZero) {
    pid_set_ki(&pid, 1.0f);
    pid_set_integral_range(&pid, 0.0f, 0.0f);
    pid_set_setpoint(&pid, 10.0f);

    for (int i = 0; i < 10; ++i) {
        pid_update(&pid, 0.0f, 0.1f);
    }

    EXPECT_FLOAT_EQ(pid.error_integral, 0.0f);
}

// T-PID-I-005a: pid_reset NOT IMPLEMENTED — expected to fail to compile (TDD red)
TEST_F(PIDTest, ResetClearsIntegralState) {
    pid_set_ki(&pid, 1.0f);
    pid_set_setpoint(&pid, 10.0f);

    for (int i = 0; i < 10; ++i) {
        pid_update(&pid, 0.0f, 0.1f);
    }

    pid_reset(&pid);

    EXPECT_FLOAT_EQ(pid.error_integral, 0.0f);
    EXPECT_FLOAT_EQ(pid.input_derivative, 0.0f);
    EXPECT_FLOAT_EQ(pid.prev_input, 0.0f);
}

// ----------------------------------------------------------------------------
// Derivative term — T-PID-D-001b, T-PID-D-002a, T-PID-D-003a, T-PID-D-004a/b
// ----------------------------------------------------------------------------

TEST_F(PIDTest, DerivativeDoesNotKickOnSetpointChange) {
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.0f);
    pid_set_setpoint(&pid, 0.0f);

    pid_update(&pid, 5.0f, 0.1f);  // input changes: prev_input 0 -> 5
    float derivative_before = pid.input_derivative;

    pid_set_setpoint(&pid, 100.0f);  // large setpoint jump; input stays at 5
    pid_update(&pid, 5.0f, 0.1f);
    float derivative_after = pid.input_derivative;

    // Derivative depends only on input changes, not setpoint changes
    EXPECT_FLOAT_EQ(derivative_after, derivative_before);
}

TEST_F(PIDTest, DerivativeIsFilteredWithTau) {
    // tau=0: step input 0 -> 10 at dt=0.1 -> large derivative
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.0f);
    pid_update(&pid, 0.0f, 0.1f);
    pid_update(&pid, 10.0f, 0.1f);
    float deriv_unfiltered = fabsf(pid.input_derivative);

    // tau=0.5: same step -> smaller derivative (filtered)
    pid_init(&pid);
    pid_set_output_range(&pid, -1000.0f, 1000.0f);
    pid_set_integral_range(&pid, -1000.0f, 1000.0f);
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.5f);
    pid_update(&pid, 0.0f, 0.1f);
    pid_update(&pid, 10.0f, 0.1f);
    float deriv_filtered = fabsf(pid.input_derivative);

    EXPECT_LT(deriv_filtered, deriv_unfiltered);
}

// NOTE: With tau=0 the Tustin transform yields an effective factor of 2*kd.
// D = -(2*kd*(input[n]-input[n-1]) + (2*tau-dt)*D[n-1]) / (2*tau+dt)
// When tau=0: D = -(2*kd*delta_input) / dt  (first step, D_prev=0)
TEST_F(PIDTest, DerivativeWithZeroTauIsScaledBackwardDifference) {
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.0f);

    pid_update(&pid, 0.0f, 0.1f);   // prev_input=0, D_prev=0
    pid_update(&pid, 10.0f, 0.1f);  // step from 0 to 10

    // D = -(2 * 1 * 10) / 0.1 = -200
    EXPECT_NEAR(pid.input_derivative, -200.0f, 1e-2f);
}

TEST_F(PIDTest, IncreasingInputGivesNegativeDerivative) {
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.0f);

    pid_update(&pid, 0.0f, 0.1f);
    pid_update(&pid, 5.0f, 0.1f);  // increasing input

    EXPECT_LT(pid.input_derivative, 0.0f);
}

TEST_F(PIDTest, DecreasingInputGivesPositiveDerivative) {
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.5f);  // tau > 0 so derivative decays to 0 with constant input

    // Warm up: constant input until derivative settles to near 0
    for (int i = 0; i < 50; ++i) {
        pid_update(&pid, 5.0f, 0.1f);
    }

    pid_update(&pid, 3.0f, 0.1f);  // decreasing input

    EXPECT_GT(pid.input_derivative, 0.0f);
}

// ----------------------------------------------------------------------------
// Output — T-PID-OUT-001a, T-PID-OUT-002b, T-PID-OUT-003a
// ----------------------------------------------------------------------------

TEST_F(PIDTest, OutputIsSumOfPIDTerms) {
    // kp=2, ki=1, kd=0, setpoint=10, input=7, dt=1.0
    // error=3; P=6; I=3*1.0=3 -> ki*I=3; D=0 -> output=9
    pid_set_kp(&pid, 2.0f);
    pid_set_ki(&pid, 1.0f);
    pid_set_setpoint(&pid, 10.0f);

    float output = pid_update(&pid, 7.0f, 1.0f);

    EXPECT_NEAR(output, 9.0f, 1e-4f);
}

TEST_F(PIDTest, OutputClampedAtNegativeEnd) {
    pid_set_kp(&pid, 10.0f);
    pid_set_output_range(&pid, -5.0f, 5.0f);
    pid_set_setpoint(&pid, -100.0f);

    float output = pid_update(&pid, 0.0f, 0.1f);

    EXPECT_FLOAT_EQ(output, -5.0f);
}

TEST_F(PIDTest, DefaultRangeGivesZeroOutput) {
    pid_controller_t pid_default;
    pid_init(&pid_default);

    pid_set_kp(&pid_default, 10.0f);
    pid_set_ki(&pid_default, 10.0f);
    pid_set_kd(&pid_default, 10.0f);
    pid_set_setpoint(&pid_default, 100.0f);

    float output = pid_update(&pid_default, 0.0f, 0.1f);

    EXPECT_FLOAT_EQ(output, 0.0f);
}

// ----------------------------------------------------------------------------
// Edge cases — T-PID-EDGE-002a, T-PID-EDGE-003a/b
// ----------------------------------------------------------------------------

TEST_F(PIDTest, ZeroTauZeroDeltaTimeIsFinite) {
    pid_set_kd(&pid, 1.0f);
    pid_set_tau(&pid, 0.0f);
    pid_set_setpoint(&pid, 5.0f);

    float output = pid_update(&pid, 1.0f, 0.0f);

    EXPECT_TRUE(std::isfinite(output));
    EXPECT_FLOAT_EQ(pid.input_derivative, 0.0f);
}

// T-PID-EDGE-003a: NaN input handling NOT IMPLEMENTED — expected to fail
TEST_F(PIDTest, NaNInputGivesFiniteOutput) {
    pid_set_kp(&pid, 1.0f);
    pid_set_setpoint(&pid, 5.0f);

    float output = pid_update(&pid, std::numeric_limits<float>::quiet_NaN(), 0.1f);

    EXPECT_TRUE(std::isfinite(output));
}

// T-PID-EDGE-003b: Inf input handling NOT IMPLEMENTED — expected to fail
TEST_F(PIDTest, InfInputGivesFiniteOutput) {
    pid_set_kp(&pid, 1.0f);
    pid_set_setpoint(&pid, 5.0f);

    float output = pid_update(&pid, std::numeric_limits<float>::infinity(), 0.1f);

    EXPECT_TRUE(std::isfinite(output));
}
