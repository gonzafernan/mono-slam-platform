/**
 * @file stm32f4_encoder_config.h
 * @brief Configuration file for STM32F4 encoder settings.
 */

#ifndef STM32F4_ENCODER_CONFIG_H
#define STM32F4_ENCODER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channels;
} stm32f4_encoder_handle_t;

#ifdef __cplusplus
}
#endif

#endif  // STM32F4_ENCODER_CONFIG_H