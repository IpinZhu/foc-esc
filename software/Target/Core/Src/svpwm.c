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

/// @brief 七段式Svpwm生成函数
/// @param ualpha 期望alpha电压
/// @param ubeta 期望beta电压
/// @param Udc  采样得到Udc
void Svpwm(float32_t ualpha, float32_t ubeta, float32_t Udc)
{
    float32_t WX, WY, WZ, WW;
    uint16_t Tx, Ty, T0;
    uint16_t chan1, chan2, chan3; // 定时器上占空比

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