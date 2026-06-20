// components/ir_array.hpp
// N reflective IR pairs (emitter GPIO + receiver on ADC). Time-multiplexed:
// only one emitter is ever lit at a time. Passive test validates presence
// (ambient plausibility), and -- only if emitters are enabled AND resistors
// are installed -- the lit delta and cross-pair isolation.
#pragma once
#include "selftest.h"
#include "pins.h"

class IrArray : public ISelfTest {
public:
    void init();

    uint16_t readReceiver(uint8_t pair);            // single ADC sample
    void     emitter(uint8_t pair, bool on);
    uint16_t readAmbient(uint8_t pair);
    uint16_t readLit(uint8_t pair);                 // fire + sample + off

    const char* name() const override { return "IR"; }
    Component component() const override { return Component::IrPair; }

    StatusCode selftestPassive() override;

private:
    uint16_t adcRead(uint32_t channel);
};