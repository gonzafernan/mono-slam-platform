/**
 * @file stm32f4_debug.c
 * @brief Debug interface for STM32F4 platform.
 */

#include <stdint.h>
#include "usart.h"

UART_HandleTypeDef *huart_debug;

void debug_init(void *context) { huart_debug = (UART_HandleTypeDef *)context; }

int __io_putchar(int ch) {
    HAL_UART_Transmit(huart_debug, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
