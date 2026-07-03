#include "drivetrain_globals.h"
#include "drivetrain.h"
#include "pins.h"
#include "globals.h"
#include "logger.h"
#include "debug_io.h"
#include "stm32f4xx_hal.h"

static int16_t absValue(int16_t value)
{
    if (value < 0)
    {
        return int16_t(-value);
    }

    return value;
}

static bool writePin(const GpioPin &pin, bool high)
{
    GPIO_PinState state = GPIO_PIN_RESET;

    if (high == true)
    {
        state = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pin.p_port, pin.pin, state);

    return true;
}

bool Drivetrain::initialize()
{
    int side = 0;

    for (side = 0; side < MOTOR_SIDE_COUNT; side++)
    {
        HAL_TIM_PWM_Start(Pins::MOTOR_PWM_TIM, Pins::MOTOR_PWM_CH[side]);
        HAL_TIM_Encoder_Start(Pins::ENC_TIM[side], TIM_CHANNEL_ALL);
        coast(uint8_t(side));
    }

    setEnabled(false);

    return true;
}

const char *Drivetrain::getName()
{
    return "Drivetrain";
}

ComponentId Drivetrain::getComponent()
{
    return ComponentId::DRIVETRAIN;
}

bool Drivetrain::setEnabled(bool enabled)
{
    return writePin(Pins::MOTOR_STBY, enabled);
}

bool Drivetrain::drive(uint8_t side, int16_t duty)
{
    if (side >= MOTOR_SIDE_COUNT)
    {
        return false;
    }

    int16_t limit = int16_t(Pins::MOTOR_PWM_MAX);

    if (duty > limit)
    {
        duty = limit;
    }

    if (duty < int16_t(-limit))
    {
        duty = int16_t(-limit);
    }

    bool    forward = (duty >= 0);
    int16_t mag     = absValue(duty);

    writePin(Pins::MOTOR_IN1[side], forward);
    writePin(Pins::MOTOR_IN2[side], (forward == false));
    __HAL_TIM_SET_COMPARE(Pins::MOTOR_PWM_TIM, Pins::MOTOR_PWM_CH[side], uint32_t(mag));

    return true;
}

bool Drivetrain::coast(uint8_t side)
{
    if (side >= MOTOR_SIDE_COUNT)
    {
        return false;
    }

    writePin(Pins::MOTOR_IN1[side], false);
    writePin(Pins::MOTOR_IN2[side], false);
    __HAL_TIM_SET_COMPARE(Pins::MOTOR_PWM_TIM, Pins::MOTOR_PWM_CH[side], 0);

    return true;
}

bool Drivetrain::resetEncoder(uint8_t side)
{
    if (side >= MOTOR_SIDE_COUNT)
    {
        return false;
    }

    m_encStart[side] = uint16_t(__HAL_TIM_GET_COUNTER(Pins::ENC_TIM[side]));

    return true;
}

bool Drivetrain::readEncoderDelta(uint8_t side, int16_t &out_counts)
{
    if (side >= MOTOR_SIDE_COUNT)
    {
        return false;
    }

    uint16_t now = uint16_t(__HAL_TIM_GET_COUNTER(Pins::ENC_TIM[side]));
    out_counts = int16_t(now - m_encStart[side]);

    return true;
}

bool Drivetrain::runPassiveTest(StatusCode &out_status)
{
    int     side = 0;
    int16_t counts = 0;

    setEnabled(false);

    for (side = 0; side < MOTOR_SIDE_COUNT; side++)
    {
        resetEncoder(uint8_t(side));
    }

    HAL_Delay(50);

    for (side = 0; side < MOTOR_SIDE_COUNT; side++)
    {
        readEncoderDelta(uint8_t(side), counts);

        if (absValue(counts) > MOTOR_NOISE_COUNTS)
        {
            out_status = makeStatus(gThisBoard, ComponentId::ENCODER, uint8_t(side), Fault::OUT_OF_RANGE);
            return false;
        }
    }

    out_status = makeOk(gThisBoard, ComponentId::DRIVETRAIN, 0);

    return true;
}

bool Drivetrain::testOneMove(uint8_t side, bool forward, StatusCode &out_status)
{
    int16_t  test_duty = MOTOR_TEST_DUTY;
    int16_t  kick_duty = MOTOR_KICK_DUTY;
    int16_t  counts    = 0;
    uint32_t start     = 0;

    if (forward == false)
    {
        test_duty = int16_t(-test_duty);
        kick_duty = int16_t(-kick_duty);
    }

    resetEncoder(side);
    setEnabled(true);
    drive(side, kick_duty);
    HAL_Delay(MOTOR_KICK_MS);
    drive(side, test_duty);
    HAL_Delay(MOTOR_TEST_MS);

    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < MOTOR_TIMEOUT_MS)
    {
        readEncoderDelta(side, counts);

        if (absValue(counts) >= MOTOR_MIN_COUNTS)
        {
            break;
        }
    }

    coast(side);
    HAL_Delay(MOTOR_SETTLE_MS);
    readEncoderDelta(side, counts);

    gLogger.info("Drivetrain move side:%u forward:%u counts:%d", side, uint8_t(forward), counts);

    if (absValue(counts) < MOTOR_NOISE_COUNTS)
    {
        out_status = makeStatus(gThisBoard, ComponentId::MOTOR, side, Fault::NO_CHANGE);
        return false;
    }

    if (absValue(counts) < MOTOR_MIN_COUNTS)
    {
        out_status = makeStatus(gThisBoard, ComponentId::MOTOR, side, Fault::UNDERPERFORM);
        return false;
    }

    bool counted_forward = (counts > 0);

    if (counted_forward != forward)
    {
        out_status = makeStatus(gThisBoard, ComponentId::ENCODER, side, Fault::WRONG_DIRECTION);
        return false;
    }

    out_status = makeOk(gThisBoard, ComponentId::MOTOR, side);

    setEnabled(false);

    return true;
}

bool Drivetrain::runInteractiveTest(StatusCode &out_status)
{
    StatusCode result;
    StatusCode worst = makeOk(gThisBoard, ComponentId::DRIVETRAIN, 0);

    if (gDebugBuild == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::DRIVETRAIN, 0, Fault::NOT_TESTED);
        return false;
    }

    gLogger.info("Drivetrain motion test, lift the wheels off the surface");
    setEnabled(true);

    debugPrompt("Next: left motor forward");
    testOneMove(MOTOR_SIDE_LEFT, true, result);
    combineWorst(worst, result);

    debugPrompt("Next: left motor reverse");
    testOneMove(MOTOR_SIDE_LEFT, false, result);
    combineWorst(worst, result);

    debugPrompt("Next: right motor forward");
    testOneMove(MOTOR_SIDE_RIGHT, true, result);
    combineWorst(worst, result);

    debugPrompt("Next: right motor reverse");
    testOneMove(MOTOR_SIDE_RIGHT, false, result);
    combineWorst(worst, result);

    setEnabled(false);

    out_status = worst;

    return worst.isOk();
}
