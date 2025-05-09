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
#include "imu.h"
#include "usart.h"


#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>


#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>


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
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;

    bool imu_status = imu_is_device_available();

    // micro-ROS configuration

    rmw_uros_set_custom_transport(
        true, (void*)&huart1, cubemx_transport_open, cubemx_transport_close,
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

    rcl_publisher_t publisher;
    sensor_msgs__msg__Imu msg;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cubemx_node", "", &support);

    // create publisher
    rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_publisher");

    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    msg.header.frame_id.data = "imu_frame";
    msg.header.frame_id.size = strlen(msg.header.frame_id.data);
    msg.header.frame_id.capacity = msg.header.frame_id.size + 1;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.orientation_covariance[0] = 0.0;
    msg.orientation_covariance[1] = 0.0;
    msg.orientation_covariance[2] = 0.0;
    msg.orientation_covariance[3] = 0.0;
    msg.orientation_covariance[4] = 0.0;
    msg.orientation_covariance[5] = 0.0;
    msg.orientation_covariance[6] = 0.0;
    msg.orientation_covariance[7] = 0.0;
    msg.orientation_covariance[8] = 0.0;
    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;
    msg.angular_velocity_covariance[0] = 0.0;
    msg.angular_velocity_covariance[1] = 0.0;
    msg.angular_velocity_covariance[2] = 0.0;
    msg.angular_velocity_covariance[3] = 0.0;
    msg.angular_velocity_covariance[4] = 0.0;
    msg.angular_velocity_covariance[5] = 0.0;
    msg.angular_velocity_covariance[6] = 0.0;
    msg.angular_velocity_covariance[7] = 0.0;
    msg.angular_velocity_covariance[8] = 0.0;
    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 0.0;
    msg.linear_acceleration_covariance[0] = 0.0;
    msg.linear_acceleration_covariance[1] = 0.0;
    msg.linear_acceleration_covariance[2] = 0.0;
    msg.linear_acceleration_covariance[3] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.0;
    msg.linear_acceleration_covariance[5] = 0.0;
    msg.linear_acceleration_covariance[6] = 0.0;
    msg.linear_acceleration_covariance[7] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.0;

    double accel_x_d = 0.0, accel_y_d = 0.0, accel_z_d = 0.0;
    double gyro_x_d, gyro_y_d, gyro_z_d;

    for (;;) {
        rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Error publishing (line %d)\n", __LINE__);
        }

        if (imu_read_accelerometer(&accel_x, &accel_y, &accel_z)) {
            msg.linear_acceleration.x = 0.0;
            msg.linear_acceleration.y = 0.0;
            msg.linear_acceleration.z = 0.0;
        } else {
            accel_x_d = (double)accel_x / 16384.0;
            accel_y_d = (double)accel_y / 16384.0;
            accel_z_d = (double)accel_z / 16384.0;
            msg.linear_acceleration.x = accel_x_d;
            msg.linear_acceleration.y = accel_y_d;
            msg.linear_acceleration.z = accel_z_d;
        }

        if (imu_read_gyroscope(&gyro_x, &gyro_y, &gyro_z)) {
            msg.angular_velocity.x = 0.0;
            msg.angular_velocity.y = 0.0;
            msg.angular_velocity.z = 0.0;
        } else {
            msg.angular_velocity.x = (double)gyro_x / 131.0;
            msg.angular_velocity.y = (double)gyro_y / 131.0;
            msg.angular_velocity.z = (double)gyro_z / 131.0;
        }

        if (imu_read_magnetometer(&mag_x, &mag_y, &mag_z)) {
            msg.linear_acceleration_covariance[0] = 0.0;
            msg.linear_acceleration_covariance[1] = 0.0;
            msg.linear_acceleration_covariance[3] = 0.0;
        } else {
            msg.linear_acceleration_covariance[0] =
                (double)mag_x;
            msg.linear_acceleration_covariance[1] =
                (double)mag_y;
            msg.linear_acceleration_covariance[2] =
                (double)mag_z;
        }
        // msg.orientation.x = get_roll_from_accel(accel_x_d, accel_y_d,
        // accel_z_d); msg.orientation.y = get_pitch_from_accel(accel_x_d,
        // accel_y_d, accel_z_d); HAL_GPIO_TogglePin(USER_LED_GPIO_Port,
        // USER_LED_Pin);
        osDelay(100);
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) {
        imu_task_notify_from_isr();
    }
}

/* USER CODE END Application */

