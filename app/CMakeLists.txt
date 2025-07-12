cmake_minimum_required(VERSION 3.10)
project(slam_platform C CXX)

enable_testing()

add_library(pid STATIC
    src/pid.c
)

target_include_directories(pid PUBLIC include)

# Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(test_pid
    test/test_pid.cpp
)

target_link_libraries(test_pid
    pid
    gtest_main
)

target_include_directories(test_pid PRIVATE include test)

include(GoogleTest)
gtest_discover_tests(test_pid)