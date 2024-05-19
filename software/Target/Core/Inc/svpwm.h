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
#include "constant.h"
#include "tim.h"

#define SVPWM_FLOAT_ENABLE 1
#define SVPWM_Q32_ENABLE 0

extern TIM_HandleTypeDef htim1;

#if SVPWM_FLOAT_ENABLE

typedef struct
{
    float32_t Ia;
    float32_t Ib;
    float32_t Ic;
} Curr_Samp; // 电流采样值
typedef struct
{
    float32_t sI1;
    float32_t sI2;
} Curr_Comp; // 电流值结构体

typedef struct
{
    float32_t sV1;
    float32_t sV2;
} Volt_Comp; // 电压值结构体
typedef struct
{
    float32_t hCos;
    float32_t hSin;
} Trig_Comp; // 角度值结构提

typedef struct
{
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;
    float32_t Limit_Integral_Max; // 积分上限
    float32_t Limit_Integral_Min; // 积分下限
    float32_t Integral_Sum;       // 积分和
    float32_t Diff_Latest;        // 这一次的差分值
    float32_t Previous_Diff;      // 上一次的误差
} PID_Struct;                     // PID参数

#endif

void Svpwm(float32_t ualpha, float32_t ubeta, float32_t Udc);
int32_t Sqrt(int32_t wInput);
Curr_Comp Clarke(Curr_Samp current);
Curr_Comp Park(Curr_Comp transcurrent, Trig_Comp zeta);
Trig_Comp SinCosZeta(float32_t zeta);
Volt_Comp Inv_Park(Volt_Comp uduq, Trig_Comp zeta);
void Openloop_Init();
void Openloop_Step();

#endif // !_SVPWM_H
