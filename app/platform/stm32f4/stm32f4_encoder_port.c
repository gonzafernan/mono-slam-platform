/**
 * @file stm32f4_encoder_port.c
 * @brief Encoder module for handling encoder operations.
 */

#include "stm32f4_encoder_port_config.h"
#include "tim.h"

int encoder_port_init(void *encoder_handle) {
    stm32f4_encoder_handle_t *enc_handle =
        (stm32f4_encoder_handle_t *)encoder_handle;
    HAL_StatusTypeDef status =
        HAL_TIM_Encoder_Start(enc_handle->htim, enc_handle->channels);
    return (status == HAL_OK) ? 0 : -1;
}

uint32_t encoder_port_sample(void *encoder_handle) {
    stm32f4_encoder_handle_t *enc_handle =
        (stm32f4_encoder_handle_t *)encoder_handle;
    return (uint32_t)enc_handle->htim->Instance->CNT;
}
