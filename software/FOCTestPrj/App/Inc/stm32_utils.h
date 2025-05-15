#pragma once

#include "main.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define i8 int8_t
#define i16 int16_t
#define i32 int32_t
#define f32 float
#define f64 double

// PWM generator paramenters
static constexpr u16 PRESCALER = 2;
static constexpr u16 TIM_CLOCK_FREQUENCY = 480;
static constexpr u16 COUNTER_PERIOD = 1000;
static constexpr u16 PWM_FREQUENCY =
    TIM_CLOCK_FREQUENCY / (PRESCALER - 1) / (COUNTER_PERIOD - 1);
