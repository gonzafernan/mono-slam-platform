/**
 * @file mock_hardware.cpp
 * @brief Mock implementation of the hardware functions for testing.
 */

#include <stdint.h>

#include "mock_hardware.hpp"

MockHardware *MockHardware::instance = nullptr;

extern "C" {

int encoder_port_init(void *encoder_handle) {
    if (MockHardware::instance) {
        return MockHardware::instance->encoder_port_init(encoder_handle);
    }
    return 0;
}

uint32_t encoder_port_sample(void *encoder_handle) {
    if (MockHardware::instance) {
        return MockHardware::instance->encoder_port_sample(encoder_handle);
    }
    return 0;
}
}
