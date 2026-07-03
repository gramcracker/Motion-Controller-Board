#pragma once

#include "self_test.h"
#include <cstdint>

class Power : public SelfTest
{
public:
    bool        initialize() override;
    const char *getName() override;
    ComponentId getComponent() override;

    bool runPassiveTest(StatusCode &out_status) override;

    bool measureVdda(float &out_vdda);
    bool measureVbat(float &out_vbat);

private:
    float m_vdda = 3.3f;
};
