#include "ir_array_globals.h"
#include "ir_array.h"
#include "adc_util.h"
#include "globals.h"
#include "logger.h"
#include "debug_io.h"
#include "stm32f4xx_hal.h"
#include <cstdio>

static bool busyDelayUs(uint32_t microseconds)
{
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000U) / 4U;

    while (cycles > 0)
    {
        cycles = cycles - 1;
        __NOP();
    }

    return true;
}

bool IrArray::initialize()
{
    uint8_t pair = 0;

    for (pair = 0; pair < IR_PAIR_COUNT; pair++)
    {
        setEmitter(pair, false);
    }

    return true;
}

const char *IrArray::getName()
{
    return "IrArray";
}

ComponentId IrArray::getComponent()
{
    return ComponentId::IR_PAIR;
}

bool IrArray::setEmitter(uint8_t pair, bool on)
{
    if (pair >= IR_PAIR_COUNT)
    {
        return false;
    }

    GPIO_PinState state = GPIO_PIN_RESET;

    if (on == true)
    {
        state = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(IR_EMITTER_PINS[pair].p_port, IR_EMITTER_PINS[pair].pin, state);

    return true;
}

bool IrArray::readAmbient(uint8_t pair, uint16_t &out_value)
{
    if (pair >= IR_PAIR_COUNT)
    {
        return false;
    }

    setEmitter(pair, false);
    busyDelayUs(IR_SETTLE_US);

    return adcReadChannel(IR_RECEIVER_CHANNELS[pair], IR_SAMPLE_TIME, out_value);
}

bool IrArray::readLit(uint8_t pair, uint16_t &out_value)
{
    if (pair >= IR_PAIR_COUNT)
    {
        return false;
    }

    setEmitter(pair, true);
    busyDelayUs(IR_SETTLE_US);
    bool ok = adcReadChannel(IR_RECEIVER_CHANNELS[pair], IR_SAMPLE_TIME, out_value);
    setEmitter(pair, false);

    return ok;
}

bool IrArray::checkPair(uint8_t pair, StatusCode &out_status)
{
    uint16_t ambient = 0;
    uint16_t lit     = 0;

    if (readAmbient(pair, ambient) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_RECEIVER, pair, Fault::NO_RESPONSE);
        return false;
    }

    if (ambient <= IR_AMBIENT_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_RECEIVER, pair, Fault::STUCK_LOW);
        return false;
    }

    if (ambient >= IR_AMBIENT_MAX)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_RECEIVER, pair, Fault::STUCK_HIGH);
        return false;
    }

    if (IR_EMITTERS_ENABLED == 0)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_EMITTER, pair, Fault::NOT_TESTED);
        return false;
    }

    if (IR_HAS_RESISTORS == 0)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_EMITTER, pair, Fault::NOT_TESTED);
        return false;
    }

    if (readLit(pair, lit) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_RECEIVER, pair, Fault::NO_RESPONSE);
        return false;
    }

    gLogger.info("IrArray pair:%u ambient:%u lit:%u", pair, ambient, lit);

    if (int(lit) - int(ambient) < IR_DELTA_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_PAIR, pair, Fault::NO_CHANGE);
        return false;
    }

    out_status = makeOk(gThisBoard, ComponentId::IR_PAIR, pair);

    return true;
}

bool IrArray::runPassiveTest(StatusCode &out_status)
{
    uint8_t    pair = 0;
    StatusCode result;
    StatusCode worst = makeOk(gThisBoard, ComponentId::IR_PAIR, 0);

    for (pair = 0; pair < IR_PAIR_COUNT; pair++)
    {
        checkPair(pair, result);
        combineWorst(worst, result);
    }

    out_status = worst;

    return worst.isOk();
}

bool IrArray::runInteractiveTest(StatusCode &out_status)
{
    uint8_t    pair = 0;
    char       message[48] = {0};
    StatusCode result;
    StatusCode worst = makeOk(gThisBoard, ComponentId::IR_PAIR, 0);

    if (gDebugBuild == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IR_PAIR, 0, Fault::NOT_TESTED);
        return false;
    }

    for (pair = 0; pair < IR_PAIR_COUNT; pair++)
    {
        snprintf(message, sizeof(message), "Next: IR pair %u, place a surface in front", pair);
        debugPrompt(message);

        checkPair(pair, result);
        combineWorst(worst, result);
    }

    out_status = worst;

    return worst.isOk();
}