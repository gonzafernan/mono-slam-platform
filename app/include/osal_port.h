/**
 * @file osal_port.h
 * @brief OSAL (Operating System Abstraction Layer) header file
 *
 * This file defines the public interface for the OSAL, which provides an
 * abstraction layer for operating system-specific functionality.
 *
 */

#ifndef OSAL_PORT_H
#define OSAL_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define OSAL_MAX_DELAY 0xFFFFFFFFUL  /// Maximum delay value for blocking calls

/**
 * @brief Create a task with static allocation.
 * @param task_function Pointer to the task function.
 * @param task_args Pointer to the arguments for the task function.
 * @param task_attributes Pointer to the task attributes structure.
 * @return Handle to the created task.
 */
void *osal_task_static_create(void (*task_function)(void *), void *task_args,
                              void *task_attributes);

/**
 * @brief Notify a task from an ISR (Interrupt Service Routine).
 * @param task_handle Handle to the task to notify.
 * @note This function should be called from an ISR context.
 */
void osal_task_notify_from_isr(void *task_handle);

/**
 * @brief Wait for a task notification.
 * @param timeout Timeout in milliseconds.
 * @note This function blocks the calling task until a notification is received
 * or the timeout expires.
 */
void osal_task_notify_wait(uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif  // OSAL_PORT_H