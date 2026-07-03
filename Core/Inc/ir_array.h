#pragma once

#include "self_test.h"
#include <cstdint>

class IrArray : public SelfTest
{
public:
    bool        initialize() override;
    const char *getName() override;
    ComponentId getComponent() override;

    bool runPassiveTest(StatusCode &out_status) override;
    bool runInteractiveTest(StatusCode &out_status) override;

    bool setEmitter(uint8_t pair, bool on);
    bool readAmbient(uint8_t pair, uint16_t &out_value);
    bool readLit(uint8_t pair, uint16_t &out_value);

private:
    bool checkPair(uint8_t pair, StatusCode &out_status);
};