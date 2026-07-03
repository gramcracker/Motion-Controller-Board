#pragma once

#include "pins.h"
#include <cstdint>

#define IR_PAIR_COUNT        3
#define IR_EMITTERS_ENABLED  0
#define IR_HAS_RESISTORS     0
#define IR_SETTLE_US         50
#define IR_AMBIENT_MIN       5
#define IR_AMBIENT_MAX       4000
#define IR_DELTA_MIN         80
#define IR_SAMPLE_TIME       ADC_SAMPLETIME_84CYCLES

inline const GpioPin IR_EMITTER_PINS[IR_PAIR_COUNT] =
{
    { GPIOB, GPIO_PIN_0 },
    { GPIOB, GPIO_PIN_8 },
    { GPIOB, GPIO_PIN_1 }
};

inline const uint32_t IR_RECEIVER_CHANNELS[IR_PAIR_COUNT] =
{
    ADC_CHANNEL_13,
    ADC_CHANNEL_14,
    ADC_CHANNEL_10
};
