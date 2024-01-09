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
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
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


void step(float32_t zeta)
{

    float32_t Id = 0, Iq = 10;
    float32_t Sinzeta, Coszeta;
    float32_t Ustep[2] = {0};

    Coszeta = arm_cos_f32(zeta);
    Sinzeta = arm_sin_f32(zeta);

    arm_inv_park_f32(Id, Iq, Ustep, Ustep + 1, Sinzeta, Coszeta);
    Svpwm(Ustep[0], Ustep[1], 10);

}