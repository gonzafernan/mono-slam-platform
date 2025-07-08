cmake_minimum_required(VERSION 3.14)
project(unit_tests)

enable_testing()
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

include_directories(../include)
add_library(pid ../src/pid.c)

add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid pid gtest_main)

include(GoogleTest)
gtest_discover_tests(test_pid)