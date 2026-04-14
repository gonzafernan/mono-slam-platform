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
 * @brief Delay execution for a specified number of milliseconds.
 * @param delay_ms Number of milliseconds to delay.
 */
void osal_delay(uint32_t delay_ms);

/**
 * @brief Loop timer for fixed-rate task scheduling.
 *
 * Maintains the last wake time internally. Call osal_loop_timer_init once
 * before the task loop, then call osal_loop_timer_wait at the top of each
 * iteration to block until the next period and obtain the real elapsed time.
 */
typedef struct {
    uint32_t last_wake_time;
    uint32_t period_ms;
} osal_loop_timer_t;

/**
 * @brief Initialize a loop timer.
 * @param timer Pointer to the loop timer.
 * @param period_ms Period in milliseconds.
 */
void osal_loop_timer_init(osal_loop_timer_t *timer, uint32_t period_ms);

/**
 * @brief Block until the next period and return the real elapsed time.
 * @param timer Pointer to the loop timer.
 * @return Elapsed time since the previous call in seconds.
 */
float osal_loop_timer_wait(osal_loop_timer_t *timer);

/**
 * @brief Get the current time in miliseconds (to be called from a task).
 * @return Current time in miliseconds.
 */
uint32_t osal_get_time_ms(void);

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

/**
 * @brief Create a queue with static allocation.
 * @param queue_length Length of the queue.
 * @param item_size Size of each item in the queue.
 * @param queue_attributes Pointer to the queue attributes structure.
 * @return Handle to the created queue.
 */
void *osal_queue_static_create(size_t queue_length, size_t item_size,
                               void *queue_attributes);

/**
 * @brief Overwrite an item in the queue.
 * @param queue_handle Handle to the queue.
 * @param item Pointer to the item to overwrite in the queue.
 * Overwrite the oldest item in the queue with the new item.
 */
void osal_queue_overwrite(void *queue_handle, void *item);

/**
 * @brief Peek at the front item of the queue without removing it.
 * @param queue_handle Handle to the queue.
 * @param item Pointer to the buffer where the item will be stored.
 * @param timeout Timeout in milliseconds for waiting for an item.
 * @return 0 if successful, -1 if timeout occurred or queue is empty.
 */
int osal_queue_peek(void *queue_handle, void *item, uint32_t timeout);

/**
 * @brief Receive and remove an item from the queue.
 * @param queue_handle Handle to the queue.
 * @param item Pointer to the buffer where the item will be stored.
 * @param timeout Timeout in milliseconds. Use OSAL_MAX_DELAY to block forever.
 * @return 0 if an item was received, -1 if timeout occurred.
 */
int osal_queue_receive(void *queue_handle, void *item, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif  // OSAL_PORT_H
