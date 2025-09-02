cmake_minimum_required(VERSION 3.10)
project(slam_platform C CXX)

enable_testing()

add_library(pid STATIC
    src/pid.c
)

add_library(exponential_filter STATIC
    src/exponential_filter.c
)

add_library(encoder STATIC
    src/encoder.c
)

target_include_directories(pid PUBLIC include)
target_include_directories(exponential_filter PUBLIC include)
target_include_directories(encoder PUBLIC include)

# Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(unittest
    test/mock_hardware.cpp
    test/test_pid.cpp
    test/test_exponential_filter.cpp
    test/test_encoder.cpp
)

target_link_libraries(unittest
    pid
    exponential_filter
    encoder
    gtest
    gmock
    gtest_main
)

target_include_directories(unittest PRIVATE include test)

include(GoogleTest)
gtest_discover_tests(unittest)
