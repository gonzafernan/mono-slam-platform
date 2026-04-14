/**
 * @file mock_osal_port.c
 * @brief Mock OSAL (Operating System Abstraction Layer) port implementation
 *
 * This file contains a mock implementation of the OSAL (Operating System
 * Abstraction Layer) for testing purposes.
 *
 */

void osal_task_notify_from_isr(void *task_handle) {}

int osal_queue_receive(void *queue_handle, void *item, unsigned int timeout) {
    return -1;
}
