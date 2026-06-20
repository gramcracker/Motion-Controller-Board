// components/ir_array.cpp
#include "ir_array.hpp"
#include "stm32f4xx_hal.h"

static inline void usDelay(uint32_t us) {
    // Coarse busy-wait; fine for a ~50us emitter settle. Replace with a DWT
    // delay if you want precision.
    uint32_t n = us * (SystemCoreClock / 1000000U) / 4U;
    while (n--) __NOP();
}

void IrArray::init() {
    for (uint8_t i = 0; i < CFG_IR_PAIR_COUNT; ++i) emitter(i, false);
}

// On-demand single-channel conversion (ADC1 shared across many sensors).
uint16_t IrArray::adcRead(uint32_t channel) {
    ADC_ChannelConfTypeDef cfg {};
    cfg.Channel = channel;
    cfg.Rank = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES;   // high-Z source
    HAL_ADC_ConfigChannel(pins::ADC, &cfg);
    HAL_ADC_Start(pins::ADC);
    uint16_t v = 0;
    if (HAL_ADC_PollForConversion(pins::ADC, 5) == HAL_OK)
        v = (uint16_t)HAL_ADC_GetValue(pins::ADC);
    HAL_ADC_Stop(pins::ADC);
    return v;
}

uint16_t IrArray::readReceiver(uint8_t pair) {
    return adcRead(pins::IR_RX_CH[pair]);
}

void IrArray::emitter(uint8_t pair, bool on) {
    const GpioPin& p = pins::IR_TX[pair];
    HAL_GPIO_WritePin(p.port, p.pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint16_t IrArray::readAmbient(uint8_t pair) {
    emitter(pair, false);
    usDelay(pins::IR_SETTLE_US);
    return readReceiver(pair);
}

uint16_t IrArray::readLit(uint8_t pair) {
    emitter(pair, true);
    usDelay(pins::IR_SETTLE_US);
    uint16_t v = readReceiver(pair);
    emitter(pair, false);
    return v;
}

StatusCode IrArray::selftestPassive() {
    StatusCode acc = OK(THIS_BOARD, Component::IrPair);

    for (uint8_t i = 0; i < CFG_IR_PAIR_COUNT; ++i) {
        // 1) Presence/plausibility from ambient -- runs regardless of emitters.
        uint16_t amb = readAmbient(i);
        if (amb <= pins::IR_AMBIENT_MIN)
            return StatusCode(THIS_BOARD, Component::IrReceiver, i, Fault::StuckLow);
        if (amb >= pins::IR_AMBIENT_MAX)
            return StatusCode(THIS_BOARD, Component::IrReceiver, i, Fault::StuckHigh);

#if CFG_ENABLE_IR && CFG_IR_EMITTERS_ENABLED && CFG_IR_HAS_RESISTORS
        // 2) Lit delta -- only when it is safe to fire the emitter.
        uint16_t lit = readLit(i);
        int diff = (int)lit - (int)amb;
        if (diff < (int)pins::IR_DELTA_MIN)
            worst(acc, StatusCode(THIS_BOARD, Component::IrPair, i, Fault::NoChange));

        // 3) Cross-pair isolation: fire pair i, neighbours must stay near ambient.
        emitter(i, true);
        usDelay(pins::IR_SETTLE_US);
        for (uint8_t j = 0; j < CFG_IR_PAIR_COUNT; ++j) {
            if (j == i) continue;
            uint16_t ja = readAmbient(j);     // (emitter j off) baseline
            // crude: compare receiver j now (i lit) vs its own ambient
            uint16_t jn = readReceiver(j);
            if ((int)jn - (int)ja > (int)pins::IR_DELTA_MIN)
                worst(acc, StatusCode(THIS_BOARD, Component::IrPair, j,
                                      Fault::Crosstalk));
        }
        emitter(i, false);
#else
        // Emitters disabled or no resistors yet: ambient-only, presence proven.
        worst(acc, StatusCode(THIS_BOARD, Component::IrEmitter, i, Fault::NotTested));
#endif
    }
    return acc;
}