/**
 * @file svpwm.h
 * @author IpinZhu (zhuyiping742@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _SVPWM_H
#define _SVPWM_H

#include "main.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;


#define SQRT3 1.732051
#define TS 2000

void Svpwm(float32_t ualpha, float32_t ubeta, float32_t Udc);
int32_t Sqrt(int32_t wInput);


#endif // !_SVPWM_H
