/**
 * @file test.c
 * @author IpinZhu (zhuyiping742@gmail.com)
 * @brief Used for test certain modules
 * @version 0.1
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "test.h"

void PWM_Test()
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2 | TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);
}

void ADC_Get()
{
    uint16_t Ialpha = 0, Ibeta = 0, Udc = 0;
    HAL_ADC_Start(&hadc1);
    Ialpha = HAL_ADC_GetValue(&hadc1);
    Ibeta = HAL_ADC_GetValue(&hadc1);
    Udc = HAL_ADC_GetValue(&hadc1);

    HAL_UART_Transmit(&huart1, Ialpha, 2U, 10);
    HAL_UART_Transmit(&huart1, "\n", 2U, 10);
    HAL_UART_Transmit(&huart1, Ibeta, 2U, 10);
    HAL_UART_Transmit(&huart1, "\n", 2U, 10);
    HAL_UART_Transmit(&huart1, Udc, 2U, 10);
    HAL_UART_Transmit(&huart1, "\n", 2U, 10);
    
}