/**
 * @file freertos_osal_port_config.h
 * @brief FreeRTOS OSAL port configuration
 *
 * This file contains the configuration for the FreeRTOS OSAL (Operating System
 * Abstraction Layer) port.
 *
 */

#ifndef FREERTOS_OSAL_PORT_CONFIG_H
#define FREERTOS_OSAL_PORT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    const char *name;      /// Name of the task
    uint16_t stack_size;   /// Stack size of the task
    UBaseType_t priority;  /// Priority of the task
    StackType_t *stack;    /// Pointer to the stack
    StaticTask_t *cb_mem;  /// Pointer to the control block memory
} freertos_osal_task_static_attr_t;

typedef struct {
    StaticQueue_t *cb_mem;  /// Pointer to the control block memory
    uint8_t *queue_buffer;  /// Pointer to the queue buffer
} freertos_osal_queue_static_attr_t;

#ifdef __cplusplus
}
#endif

#endif  // FREERTOS_OSAL_PORT_CONFIG_H
