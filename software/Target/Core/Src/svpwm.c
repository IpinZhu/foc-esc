/**
 * @file svpwm.c
 * @author IpinZhu (zhuyiping742@gmail.com)
 * @brief SVPWM generator
 * @version 0.1
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "svpwm.h"

float32_t Zeta;
Volt_Comp Start_Vol;

/// @brief 七段式Svpwm生成函数
/// @param ualpha 期望alpha电压
/// @param ubeta 期望beta电压
/// @param Udc  采样得到Udc
void Svpwm(float32_t ualpha, float32_t ubeta, float32_t Udc)
{
    float32_t WX, WY, WZ, WW;
    uint16_t Tx, Ty, T0;
    uint16_t chan1 = 0, chan2 = 0, chan3 = 0; // 定时器上占空比

    // 时间计算
    WW = (SQRT3 * TS) / Udc;
    WX = WW * ualpha;
    WY = WW * ((SQRT3 / 2) * ualpha + (ubeta / 2));
    WZ = WW * (-((SQRT3 / 2) * ualpha) + (ubeta / 2));

    // 扇区判断以及工作时间
    if (WY < 0)
    {
        if (WZ < 0)
        {
            // SECTOR2
            Tx = WZ;
            Ty = WY;
            if (Tx + Ty > TS)
            {
                // 放缩防止超调
                Tx = Tx / (Tx + Ty) * TS;
                Ty = Ty / (Tx + Ty) * TS;
            }
            T0 = (TS - Tx - Ty) / 2;

            chan1 = (Ty + T0) / 2; // T2/2+T0/2
            chan2 = T0 / 2;        // T0/2
            chan3 = (TS - T0) / 2; // T0/2+T2/2+T6/2 = Ts/2 - T7/2
        }
        else // Z>=0
            if (WX >= 0)
            {
                // SECTOR3
                Tx = WX;
                Ty = -WY;
                if (Tx + Ty > TS)
                {
                    // 放缩防止超调
                    Tx = Tx / (Tx + Ty) * TS;
                    Ty = Ty / (Tx + Ty) * TS;
                }
                T0 = (TS - Tx - Ty) / 2;

                chan1 = (TS - T0) / 2;
                chan2 = T0 / 2;
                chan3 = (T0 + Tx) / 2;
            }
            else if (WX < 0)
            {
                // SECTOR4
                Tx = -WY;
                Ty = -WZ;
                if (Tx + Ty > TS)
                {
                    // 放缩防止超调
                    Tx = Tx / (Tx + Ty) * TS;
                    Ty = Ty / (Tx + Ty) * TS;
                }
                T0 = (TS - Tx - Ty) / 2;

                chan1 = (TS - T0) / 2;
                chan2 = (Ty + T0) / 2;
                chan3 = T0 / 2;
            }
    }
    else if (WY >= 0)
    {
        if (WZ < 0)
        {
            if (WX >= 0)
            {
                // SECTOR1
                Tx = -WZ;
                Ty = WX;
                if (Tx + Ty > TS)
                {
                    // 放缩防止超调
                    Tx = Tx / (Tx + Ty) * TS;
                    Ty = Ty / (Tx + Ty) * TS;
                }
                T0 = (TS - Tx - Ty) / 2;

                chan1 = T0 / 2;
                chan2 = (Tx + T0) / 2;
                chan3 = (TS - T0) / 2;
            }
            else if (WX < 0)
            {
                // SECTOR6
                Tx = -WX;
                Ty = WZ;
                if (Tx + Ty > TS)
                {
                    // 放缩防止超调
                    Tx = Tx / (Tx + Ty) * TS;
                    Ty = Ty / (Tx + Ty) * TS;
                }
                T0 = (TS - Tx - Ty) / 2;

                chan1 = T0 / 2;
                chan2 = (TS - T0) / 2;
                chan3 = (Ty + T0) / 2;
            }
        } // WZ>=0
        else if (WX < 0)
        {
            // SECTOR5
            Tx = WY;
            Ty = -WX;
            if (Tx + Ty > TS)
            {
                // 放缩防止超调
                Tx = Tx / (Tx + Ty) * TS;
                Ty = Ty / (Tx + Ty) * TS;
            }
            T0 = (TS - Tx - Ty) / 2;

            chan1 = (Tx + T0) / 2;
            chan2 = (TS - T0) / 2;
            chan3 = T0 / 2;
        }
    }

    // 作用到定时器
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, chan1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, chan2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, chan3);
}

/**
 * @brief 对输入整数求平方根，当输入是负数的时候返回0
 * @param  Input int32_t number
 * @retval int32_t Square root of Input (0 if Input<0)
 */
int32_t Sqrt(int32_t wInput)
{
    int32_t wtemprootnew;

    if (wInput > 0)
    {
        uint8_t biter = 0u;
        int32_t wtemproot;

        if (wInput <= ((int32_t)2097152))
        {
            wtemproot = ((int32_t)128);
        }
        else
        {
            wtemproot = ((int32_t)8192);
        }

        do
        {
            wtemprootnew = (wtemproot + (wInput / wtemproot)) / (int32_t)2;
            if ((wtemprootnew == wtemproot) || ((int32_t)0 == wtemproot))
            {
                biter = 6U;
            }
            else
            {
                biter++;
                wtemproot = wtemprootnew;
            }
        } while (biter < 6U);
    }
    else
    {
        wtemprootnew = (int32_t)0;
    }

    return (wtemprootnew);
}

/// @brief clarke trans function
/// @param current
/// @return Ialpha and Ibeta
Curr_Comp Clarke(Curr_Samp current)
{
    Curr_Comp Ret;
    float32_t Ibdiv2;
    float32_t Icdiv2;

    Ibdiv2 = current.Ib / 2;
    Icdiv2 = current.Ic / 2;

    Ret.sI1 = current.Ia - Ibdiv2 - Icdiv2;
    Ret.sI2 = Ibdiv2 * SQRT3 - Icdiv2 * SQRT3;

    return Ret;
}

/// @brief Park tarans function
/// @param transcurrent
/// @return Id and Iq
Curr_Comp Park(Curr_Comp transcurrent, Trig_Comp zeta)
{
    Curr_Comp Ret;

    Ret.sI1 = zeta.hCos * transcurrent.sI1 + zeta.hSin * transcurrent.sI2;
    Ret.sI2 = zeta.hCos * transcurrent.sI2 - zeta.hSin * transcurrent.sI1;

    return Ret;
}

/// @brief
/// @param zeta
/// @return return sin(zeta) and cos(zeta)
Trig_Comp SinCosZeta(float32_t zeta)
{
    Trig_Comp Ret;
    float32_t zetaRad;
    zetaRad = ((zeta * 6.28) / 360);

    Ret.hCos = arm_cos_f32(zetaRad);
    Ret.hSin = arm_sin_f32(zetaRad);

    return Ret;
}

/// @brief RePark trans function
/// @param uduq
/// @return ualpha and ubeta
Volt_Comp Inv_Park(Volt_Comp uduq, Trig_Comp zeta)
{
    Volt_Comp Ret;

    Ret.sV1 = zeta.hCos * uduq.sV1 - zeta.hSin * uduq.sV2;
    Ret.sV2 = zeta.hSin + uduq.sV1 + zeta.hCos * uduq.sV2;

    return Ret;
}

/// @brief
void Openloop_Init()
{

    Zeta = 0;
    Start_Vol.sV1 = 0;
    Start_Vol.sV2 = 5;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
}

/// @brief
void Openloop_Step()
{
    Trig_Comp p;
    Volt_Comp n;
    Zeta += 10;
    if (Zeta >= 360)
    {
        Zeta = 0;
    }
    p = SinCosZeta(Zeta);
    n = Inv_Park(Start_Vol, p);

    Svpwm(n.sV1, n.sV2, 10);
    HAL_Delay(10);
}