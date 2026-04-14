/**
 * @file freertos_osal_port.c
 * @brief FreeRTOS OSAL port implementation
 *
 * This file contains the implementation of the OSAL (Operating System
 * Abstraction Layer) for FreeRTOS. It provides functions for task management,
 * mutex handling, and semaphore handling.
 *
 */

#include "FreeRTOS.h"
#include "freertos_osal_port_config.h"
#include "osal_port.h"
#include "queue.h"
#include "task.h"

void osal_delay(uint32_t delay_ms) {
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    vTaskDelay(ticks);
}

void osal_loop_timer_init(osal_loop_timer_t *timer, uint32_t period_ms) {
    timer->last_wake_time = xTaskGetTickCount();
    timer->period_ms = period_ms;
}

float osal_loop_timer_wait(osal_loop_timer_t *timer) {
    uint32_t prev_wake = timer->last_wake_time;
    vTaskDelayUntil(&timer->last_wake_time, pdMS_TO_TICKS(timer->period_ms));
    uint32_t now = xTaskGetTickCount();
    return (float)(now - prev_wake) * portTICK_PERIOD_MS / 1000.0f;
}

uint32_t osal_get_time_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void *osal_task_static_create(void (*task_function)(void *), void *task_args,
                              void *task_attributes) {
    freertos_osal_task_static_attr_t *task_attr =
        (freertos_osal_task_static_attr_t *)task_attributes;
    TaskHandle_t xTask;
    xTask = xTaskCreateStatic(
        (TaskFunction_t)task_function, task_attr->name, task_attr->stack_size,
        task_args, task_attr->priority, task_attr->stack, task_attr->cb_mem);
    return (void *)xTask;
}

void osal_task_notify_from_isr(void *task_handle) {
    TaskHandle_t xTask = (TaskHandle_t)task_handle;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTask, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void osal_task_notify_wait(uint32_t timeout) {
    ulTaskNotifyTake(pdTRUE, timeout);
}

void *osal_queue_static_create(size_t queue_length, size_t item_size,
                               void *queue_attributes) {
    freertos_osal_queue_static_attr_t *queue_attr =
        (freertos_osal_queue_static_attr_t *)queue_attributes;
    QueueHandle_t xQueue;
    xQueue = xQueueCreateStatic(queue_length, item_size,
                                queue_attr->queue_buffer, queue_attr->cb_mem);
    return (void *)xQueue;
}

void osal_queue_overwrite(void *queue_handle, void *item) {
    QueueHandle_t xQueue = (QueueHandle_t)queue_handle;
    xQueueOverwrite(xQueue, item);
}

int osal_queue_peek(void *queue_handle, void *item, uint32_t timeout) {
    QueueHandle_t xQueue = (QueueHandle_t)queue_handle;
    return (xQueuePeek(xQueue, item, pdMS_TO_TICKS(timeout)) == pdTRUE) ? 0
                                                                        : -1;
}

int osal_queue_receive(void *queue_handle, void *item, uint32_t timeout) {
    QueueHandle_t xQueue = (QueueHandle_t)queue_handle;
    return (xQueueReceive(xQueue, item, pdMS_TO_TICKS(timeout)) == pdTRUE) ? 0
                                                                           : -1;
}
