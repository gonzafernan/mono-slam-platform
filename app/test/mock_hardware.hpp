/**
 * @file mock_hardware.hpp
 * @brief Mock implementation of the hardware functions for testint.
 *
 * This file provides mock implementations of the hardware functions for unit
 * testing purposes.
 *
 */

#ifndef MOCK_HARDWARE_HPP
#define MOCK_HARDWARE_HPP

#include <gmock/gmock.h>

class MockHardware {
   public:
    // encoder port
    MOCK_METHOD(int, encoder_port_init, (void* encoder_handle), ());
    MOCK_METHOD(uint32_t, encoder_port_sample, (void* encoder_handle), ());
    static MockHardware* instance;
};

#endif  // MOCK_HARDWARE_HPP
