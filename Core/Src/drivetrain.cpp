// components/drivetrain.cpp
#include "drivetrain.hpp"
#include "stm32f4xx_hal.h"

static inline void writePin(const GpioPin& p, bool high) {
    HAL_GPIO_WritePin(p.port, p.pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Drivetrain::init() {
    for (int i = 0; i < 2; ++i) {
        HAL_TIM_PWM_Start(pins::MOTOR_PWM_TIM, pins::MOTOR_PWM_CH[i]);
        HAL_TIM_Encoder_Start(pins::ENC_TIM[i], TIM_CHANNEL_ALL);
        coast((Side)i);
    }
    enable(false);   // safe-by-default: motors gated off until commanded
}

void Drivetrain::enable(bool on) { writePin(pins::MOTOR_STBY, on); }

void Drivetrain::drive(Side s, int16_t duty) {
    if (duty >  (int16_t)pins::MOTOR_PWM_MAX) duty =  pins::MOTOR_PWM_MAX;
    if (duty < -(int16_t)pins::MOTOR_PWM_MAX) duty = -pins::MOTOR_PWM_MAX;
    bool fwd = duty >= 0;
    uint16_t mag = fwd ? duty : -duty;
    // Sign-magnitude: one IN high, the other low, PWM on the channel.
    writePin(pins::MOTOR_IN1[s],  fwd);
    writePin(pins::MOTOR_IN2[s], !fwd);
    __HAL_TIM_SET_COMPARE(pins::MOTOR_PWM_TIM, pins::MOTOR_PWM_CH[s], mag);
}

void Drivetrain::coast(Side s) {
    writePin(pins::MOTOR_IN1[s], false);
    writePin(pins::MOTOR_IN2[s], false);
    __HAL_TIM_SET_COMPARE(pins::MOTOR_PWM_TIM, pins::MOTOR_PWM_CH[s], 0);
}

void Drivetrain::brakeAll() {
    for (int i = 0; i < 2; ++i) {
        writePin(pins::MOTOR_IN1[(Side)i], true);   // both high = short brake
        writePin(pins::MOTOR_IN2[(Side)i], true);
    }
}

uint16_t Drivetrain::encRaw(Side s) {
    return (uint16_t)__HAL_TIM_GET_COUNTER(pins::ENC_TIM[s]);
}
void Drivetrain::encoderReset(Side s) { encStart_[s] = encRaw(s); }
int16_t Drivetrain::encoderDelta(Side s) {
    return (int16_t)(encRaw(s) - encStart_[s]);  // 16-bit wrap-safe difference
}

// Passive: with motors gated off, encoders must be quiet. Movement here means
// EMI on the channels, which would silently corrupt odometry.
StatusCode Drivetrain::selftestPassive() {
    enable(false);
    for (int i = 0; i < 2; ++i) {
        encoderReset((Side)i);
    }
    HAL_Delay(50);
    for (int i = 0; i < 2; ++i) {
        int16_t d = encoderDelta((Side)i);
        if (d > 3 || d < -3)
            return StatusCode(THIS_BOARD, Component::Encoder, i, Fault::OutOfRange);
    }
    return OK(THIS_BOARD, Component::Drivetrain);
}

StatusCode Drivetrain::testMove(Side s, bool forward, IConsole& c) {
    const int16_t duty = forward ? CFG_MOTOR_TEST_DUTY : -CFG_MOTOR_TEST_DUTY;
    encoderReset(s);
    drive(s, duty);

    uint32_t t0 = HAL_GetTick();
    int16_t d = 0;
    // Spin until we've moved enough or we time out.
    while ((HAL_GetTick() - t0) < CFG_MOTOR_TIMEOUT_MS) {
        d = encoderDelta(s);
        int16_t mag = d < 0 ? -d : d;
        if (mag >= CFG_MOTOR_MIN_COUNTS &&
            (HAL_GetTick() - t0) >= CFG_MOTOR_TEST_MS) break;
    }
    coast(s);
    HAL_Delay(60);                 // let it stop, then read settled count
    d = encoderDelta(s);
    int16_t mag = d < 0 ? -d : d;

    c.logv("  %s %s -> %d counts\n", s == LEFT ? "L" : "R",
           forward ? "fwd" : "rev", d);

    if (mag < 3)
        return StatusCode(THIS_BOARD, Component::Motor, s, Fault::NoChange);
    if (mag < CFG_MOTOR_MIN_COUNTS)
        return StatusCode(THIS_BOARD, Component::Motor, s, Fault::Underperform);
    // Direction: forward must count positive (flip in CubeMX or here if not).
    bool counted_fwd = d > 0;
    if (counted_fwd != forward)
        return StatusCode(THIS_BOARD, Component::Encoder, s, Fault::WrongDirection);
    return OK(THIS_BOARD, Component::Motor, s);
}

// Interactive == the motion test. DEBUG / TEST_MOTION only (the runtime build
// never calls this). Also checks left/right mapping via the idle encoder.
StatusCode Drivetrain::selftestInteractive(IConsole& c) {
#if TEST_MOTION
    c.log("Drivetrain motion test (wheels off the ground)...\n");
    enable(true);
    StatusCode acc = OK(THIS_BOARD, Component::Drivetrain);

    for (int si = 0; si < 2; ++si) {
        Side s = (Side)si, other = (Side)(1 - si);
        for (int fwd = 1; fwd >= 0; --fwd) {
            encoderReset(other);
            worst(acc, testMove(s, fwd != 0, c));
            // Mapping check: the OTHER wheel should not have moved.
            int16_t od = encoderDelta(other);
            if ((od > CFG_MOTOR_MIN_COUNTS) || (od < -CFG_MOTOR_MIN_COUNTS)) {
                c.logv("  ! %s moved while driving %s (swapped connectors?)\n",
                       other == LEFT ? "L" : "R", s == LEFT ? "L" : "R");
                worst(acc, StatusCode(THIS_BOARD, Component::Motor, s,
                                      Fault::WrongDirection));
            }
        }
    }
    enable(false);
    return acc;
#else
    (void)c;
    return StatusCode(THIS_BOARD, Component::Drivetrain, 0, Fault::NotTested);
#endif
}