#pragma once

#include "self_test.h"
#include <cstdint>

class Cliff : public SelfTest
{
public:
    bool        initialize() override;
    const char *getName() override;
    ComponentId getComponent() override;

    bool runPassiveTest(StatusCode &out_status) override;
    bool runInteractiveTest(StatusCode &out_status) override;

    bool readValue(uint16_t &out_value);
};
