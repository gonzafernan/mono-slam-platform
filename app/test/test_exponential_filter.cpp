/**
 * @file test_exponential_filter.cpp
 * @brief Unit tests for the exponential filter module
 */

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "exponential_filter.h"
}

TEST(ExponentialFilterTest, InitializationSetsAlphaAndZeroOutput) {
    exponential_filter_t filter;
    exponential_filter_init(&filter, 0.3f);

    EXPECT_FLOAT_EQ(filter.alpha, 0.3f);
    EXPECT_TRUE(isnanf(filter.last_output));
}

TEST(ExponentialFilterTest, FirstUpdateReturnsFirstInput) {
    exponential_filter_t filter;
    exponential_filter_init(&filter, 0.5f);

    float result = exponential_filter_update(&filter, 10.0f);

    EXPECT_DOUBLE_EQ(result, 10.0f);
    EXPECT_DOUBLE_EQ(filter.last_output, 10.0f);
}

TEST(ExponentialFilterTest, SubsequentUpdateFiltersCorrectly) {
    exponential_filter_t filter;
    exponential_filter_init(&filter, 0.2f);

    exponential_filter_update(&filter, 5.0f);  // Initial input
    float result = exponential_filter_update(&filter, 9.0f);

    // expected_output = alpha * input + (1 - alpha) * last_output
    // => 0.2 * 9 + 0.8 * 5 = 1.8 + 4.0 = 5.8
    EXPECT_NEAR(result, 5.8, 1e-6);
    EXPECT_NEAR(filter.last_output, 5.8, 1e-6);
}

TEST(ExponentialFilterTest, AlphaEqualsZeroGivesConstantOutput) {
    exponential_filter_t filter;
    exponential_filter_init(&filter, 0.0f);

    exponential_filter_update(&filter, 1.0f);
    float result = exponential_filter_update(&filter, 1000.0f);

    EXPECT_DOUBLE_EQ(result, 1.0f);
}

TEST(ExponentialFilterTest, AlphaEqualsOneTracksInputDirectly) {
    exponential_filter_t filter;
    exponential_filter_init(&filter, 1.0f);

    exponential_filter_update(&filter, 3.0f);
    float result = exponential_filter_update(&filter, 7.0f);

    EXPECT_DOUBLE_EQ(result, 7.0f);
}

TEST(ExponentialFilterTest, HandlesNegativeInputs) {
    exponential_filter_t filter;
    exponential_filter_init(&filter, 0.5f);

    exponential_filter_update(&filter, -2.0f);
    float result = exponential_filter_update(&filter, -4.0f);

    EXPECT_NEAR(result, -3.0, 1e-6);
}
