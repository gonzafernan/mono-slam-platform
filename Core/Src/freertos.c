/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "transport.h"
#include "microros_transport_config.h"
#include "imu.h"
#include "usart.h"


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <rclc_parameter/rclc_parameter.h>
#include <uxr/client/transport.h>
#include <stdio.h>


#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static transport_context_t transport_context;
static rcl_timer_t joint_state_publisher_timer;
static rcl_timer_t imu_publisher_timer;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport* transport);
bool cubemx_transport_close(struct uxrCustomTransport* transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf,
                             size_t len, int timeout, uint8_t* err);

void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void* state);
void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
// micro-ROS configuration
    rmw_uros_set_custom_transport(
        true, (void*)&huart2, cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator =
        rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "vizcc_mcu", "", &support);

    // executor
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context,
                       RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 7, &allocator);
    unsigned int rcl_executor_timeout = 10;
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_executor_timeout));

    unsigned int rcl_timer_timeout = 10;
    rclc_timer_init_default(&joint_state_publisher_timer, &support, RCL_MS_TO_NS(rcl_timer_timeout),
                            publisher_timer_callback);
    rclc_executor_add_timer(&executor, &joint_state_publisher_timer);

    rcl_timer_timeout = 20;
    rclc_timer_init_default(&imu_publisher_timer, &support, RCL_MS_TO_NS(rcl_timer_timeout),
                            publisher_timer_callback);
    rclc_executor_add_timer(&executor, &imu_publisher_timer);

    // create app transport layer
    transport_context.node = &node;
    transport_context.executor = &executor;
    transport_imu_init((void *)&transport_context);
    transport_joint_state_init((void *)&transport_context);
    if (transport_command_joint_space_init((void *)&transport_context) < 0) {
        printf("ERROR: Unable to crete joint space command subscription (line %d).\r\n",
               __LINE__);
    }
    // transport_command_velocity_init((void *)&transport_context);
    // transport_parameter_server_init((void *)&transport_context);

    rclc_executor_prepare(&executor);
    rclc_executor_spin(&executor);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) {
        imu_task_notify_from_isr();
    }
}

void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == &joint_state_publisher_timer) {
        transport_joint_state_publish();
    } else if (timer == &imu_publisher_timer) {
        transport_imu_publish();
    } else {
        printf("Error joint state publisher timer callback execution without handle (line %d)\r\n",
               __LINE__);
    }
}
/* USER CODE END Application */

