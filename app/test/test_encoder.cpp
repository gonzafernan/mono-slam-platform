/**
 * @file test_encoder.cpp
 * @brief Unit tests for the encoder driver.
 */

#include <gtest/gtest.h>
#include <cmath>
#include "encoder_port.h"

extern "C" {
#include "encoder.h"
}

#include "mock_hardware.hpp"

using ::testing::Return;

class EncoderFixture : public ::testing::Test {
   protected:
    encoder_t encoder;
    MockHardware mock;
    void *encoder_context;

    void SetUp() override {
        MockHardware::instance = &mock;
        encoder_context = (void *)1;
        encoder_init(&encoder, encoder_context, 100, 0.1f);
    }

    void TearDown() override { MockHardware::instance = nullptr; }
};

TEST(EncoderTest, InitSuccess) {
    MockHardware mock;
    MockHardware::instance = &mock;

    void *encoder_context = (void *)1;

    EXPECT_CALL(mock, encoder_port_init(encoder_context)).WillOnce(Return(0));

    encoder_t encoder;
    int ret = encoder_init(&encoder, encoder_context, 10, 0.1f);

    EXPECT_EQ(ret, 0);

    MockHardware::instance = nullptr;
}

TEST(EncoderTest, InitFailure) {
    MockHardware mock;
    MockHardware::instance = &mock;

    void *encoder_context = (void *)1;

    EXPECT_CALL(mock, encoder_port_init(encoder_context)).WillOnce(Return(-1));

    encoder_t encoder;
    int ret = encoder_init(&encoder, encoder_context, 10, 0.1f);

    EXPECT_EQ(ret, -1);

    MockHardware::instance = nullptr;
}

TEST_F(EncoderFixture, SampleAndRetrieval) {
    float angular_position, angular_velocity;
    EXPECT_CALL(mock, encoder_port_sample(encoder_context))
        .WillOnce(Return(10));
    encoder_sample(&encoder, 0.01f);
    angular_position = encoder_get_angular_position(&encoder);
    EXPECT_NEAR(angular_position, 0.6283f, 1e-4);
    angular_velocity = encoder_get_angular_velocity(&encoder);
    EXPECT_NEAR(angular_velocity, 62.83f, 1e-2);
}

TEST_F(EncoderFixture, SampleInvalidDeltaTime) {
    float angular_position, angular_velocity;

    EXPECT_CALL(mock, encoder_port_sample(encoder_context))
        .WillOnce(Return(10));
    encoder_sample(&encoder, 0.0f);
    angular_position = encoder_get_angular_position(&encoder);
    EXPECT_NEAR(angular_position, 0.6283f, 1e-4);
    angular_velocity = encoder_get_angular_velocity(&encoder);
    EXPECT_NEAR(angular_velocity, 0.0f, 1e-4);

    EXPECT_CALL(mock, encoder_port_sample(encoder_context))
        .WillOnce(Return(20));
    encoder_sample(&encoder, -0.1f);
    angular_position = encoder_get_angular_position(&encoder);
    EXPECT_NEAR(angular_position, 2 * 0.6283f, 1e-4);
    angular_velocity = encoder_get_angular_velocity(&encoder);
    EXPECT_NEAR(angular_velocity, 0.0f, 1e-4);
}
