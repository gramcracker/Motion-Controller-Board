// components/drivetrain.hpp
// Two motors + two encoders behind the TB6612. Owns STBY. The motion self-test
// closes the loop: drive each motor, confirm the CORRECT encoder counts in the
// CORRECT direction by at least the expected amount, with a hard timeout, and
// always leaves STBY low.
#pragma once
#include "selftest.h"
#include "pins.h"

class Drivetrain : public ISelfTest {
public:
    enum Side { LEFT = 0, RIGHT = 1 };

    void init();                       // start PWM + encoders, force standby
    void enable(bool on);              // STBY high/low (hardware motor gate)
    void drive(Side s, int16_t duty);  // signed: +fwd / -rev, magnitude <= PWM_MAX
    void coast(Side s);
    void brakeAll();
    int16_t encoderDelta(Side s);      // signed counts since last reset
    void encoderReset(Side s);

    const char* name() const override { return "Drivetrain"; }
    Component component() const override { return Component::Drivetrain; }

    StatusCode selftestPassive() override;       // encoder noise floor at rest
    StatusCode selftestInteractive(IConsole&) override;  // == motion test

private:
    uint16_t encRaw(Side s);
    uint16_t encStart_[2] {0, 0};

    // Tests one motor in one direction; returns Ok or a Motor/Encoder fault.
    StatusCode testMove(Side s, bool forward, IConsole& c);
};