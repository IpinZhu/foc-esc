#ifndef _USER_CONSTANT_H
#define _USER_CONSTANT_H

#define HALL_ENABLE 0     // 是否启用霍尔编码器
#define DEADTIME_ENABLE 0 // 是否启用定时器死区控制
#define MATHCONST_ENABLE 1 

//定时器常量
#if 1

#define CKFRE 170000000 // 定时器时钟频率Hz
#define PWM_PRSC 0      //分频
#define PWM_FREQ 20000 // PWM频率
#define PWM_PERIOD CKFRE / (2 * PWM_FREQ * (PWM_PRSC + 1))
#define REP_RATE 1 // 重复计数出发次数，适用于中心对其模式

#endif

//数学常量
#if MATHCONST_ENABLE 

#define SQRT3 1.732051
#define TS 2000

#endif

//死区时间常量
#if DEADTIME_ENABLE

#define DEADTIME_NS 1000 // 死区时间ns
#define DEADTIME CKFRE / 1000000 / 2 * DEADTIME_NS / 1000

#endif

//霍尔传感器常量
#if HALL_ENABLE

#define POLE_PAIR_NUM 4  // 极对数
#define ENCODER_PPR 1250 // 编码器线数
#define ALIGNMENT_ANGLE 300
#define COUNTER_RESET (ALIGNMENT_ANGLE * 4 * ENCODER_PPR / 360 - 1) / POLE_PAIR_NUM
#define ICx_FILTER 8

#endif 

#endif //_USER_CONSTANT_H