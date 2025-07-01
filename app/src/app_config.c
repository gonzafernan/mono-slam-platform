/**
 * @file app_config.c
 * @brief Application configuration implementation
 */

#include "app_config.h"
#include "imu.h"

// imu task
/// Buffer for the IMU task stack
uint32_t imu_stack_buffer[IMU_TASK_STACK_SIZE];
StaticTask_t imu_task_buffer;

// imu queue
uint8_t imu_queue_buffer[sizeof(imu_sample_t)];  /// Buffer for the IMU queue
StaticQueue_t imu_queue_cbm;  /// Control block memory for the IMU queue

freertos_osal_task_static_attr_t imu_task_attr = {
    .name = "imu_task",                 /// Name of the IMU task
    .stack_size = IMU_TASK_STACK_SIZE,  /// Stack size for the IMU task
    .priority = osPriorityNormal,       /// Priority of the IMU task
    .stack = imu_stack_buffer,          /// Pointer to the stack
    .cb_mem = &imu_task_buffer,         /// Pointer to the control block memory
};

freertos_osal_queue_static_attr_t imu_queue_attr = {
    .queue_buffer = imu_queue_buffer,  /// Pointer to the queue buffer
    .cb_mem = &imu_queue_cbm,          /// Pointer to the control block memory
};

// actuator task
/// Buffer for the actuator task stack
uint32_t actuator1_stack_buffer[ACTUATOR_TASK_STACK_SIZE];
StaticTask_t actuator1_task_buffer;

freertos_osal_task_static_attr_t actuator1_task_attr = {
    .name = "actuator1_task",  /// Name of the actuator task
    .stack_size =
        ACTUATOR_TASK_STACK_SIZE,      /// Stack size for the actuator task
    .priority = osPriorityNormal,      /// Priority of the actuator task
    .stack = actuator1_stack_buffer,   /// Pointer to the stack
    .cb_mem = &actuator1_task_buffer,  /// Pointer to the control block memory
};