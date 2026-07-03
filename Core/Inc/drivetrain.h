#pragma once

#include "self_test.h"
#include <cstdint>
#include "drivetrain_globals.h"

class Drivetrain : public SelfTest
{
public:
    bool        initialize() override;
    const char *getName() override;
    ComponentId getComponent() override;

    bool runPassiveTest(StatusCode &out_status) override;
    bool runInteractiveTest(StatusCode &out_status) override;

    bool setEnabled(bool enabled);
    bool drive(uint8_t side, int16_t duty);
    bool coast(uint8_t side);
    bool resetEncoder(uint8_t side);
    bool readEncoderDelta(uint8_t side, int16_t &out_counts);

private:
    bool testOneMove(uint8_t side, bool forward, StatusCode &out_status);

    uint16_t m_encStart[MOTOR_SIDE_COUNT] = {0, 0};
};
