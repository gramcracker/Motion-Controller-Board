#pragma once

#include <cstdint>

#define CONTROL_RATE_HZ             1000
#define CONTROL_LOOP_TIM            TIM6

#define WHEEL_DIAMETER_MM           34
#define ENCODER_PPR                 7
#define ENCODER_QUADRATURE          4
#define GEAR_RATIO                  42.0f
#define TRACK_WIDTH_MM              100

#define MOTION_VEL_FILTER_OLD       0.8f
#define MOTION_VEL_FILTER_NEW       0.2f

#define ENABLE_VELOCITY_LOOP        0

#define VEL_PID_KP                  8.0f
#define VEL_PID_KI                  40.0f
#define VEL_PID_KD                  0.0f
#define VEL_PID_INTEGRAL_MAX        4199.0f