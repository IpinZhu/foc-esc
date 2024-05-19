/**
 * @file test.h
 * @author IpinZhu (zhuyiping742@gmail.com)
 * @brief Used for test certain modules
 * @version 0.1
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _TEST_H
#define _TEST_H

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "test.h"
#include "arm_math.h"
#include "svpwm.h"
#include "constant.h"

extern TIM_HandleTypeDef htim1;

extern ADC_HandleTypeDef hadc1;

extern UART_HandleTypeDef huart1;


void PWM_Test();
void ADC_Get();
void OpenLoopStep(float32_t zeta);

#endif // !_TEST_H