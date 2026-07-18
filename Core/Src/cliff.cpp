#include "cliff_globals.h"
#include "cliff.h"
#include "adc_util.h"
#include "pins.h"
#include "globals.h"
#include "logger.h"
#include "debug_io.h"
#include "stm32g4xx_hal.h"

bool Cliff::initialize()
{
    return true;
}

const char *Cliff::getName()
{
    return "Cliff";
}

ComponentId Cliff::getComponent()
{
    return ComponentId::CLIFF;
}

bool Cliff::readValue(uint16_t &out_value)
{
    return adcReadChannel(Pins::CLIFF_CH, CLIFF_SAMPLE_TIME, out_value);
}

bool Cliff::runPassiveTest(StatusCode &out_status)
{
    uint16_t value = 0;

    if (readValue(value) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::NO_RESPONSE);
        return false;
    }

    if (value == 0)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::STUCK_LOW);
        return false;
    }

    if (value >= CLIFF_RAILED_HIGH)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::STUCK_HIGH);
        return false;
    }

    if (value < CLIFF_ON_TABLE_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::OUT_OF_RANGE);
        return false;
    }

    gLogger.info("Cliff value:%u", value);

    out_status = makeOk(gThisBoard, ComponentId::CLIFF, 0);

    return true;
}

bool Cliff::runInteractiveTest(StatusCode &out_status)
{
    uint16_t value   = 0;
    uint16_t low     = 0;
    uint16_t high    = 0;
    uint32_t start   = 0;

    if (gDebugBuild == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::NOT_TESTED);
        return false;
    }

    debugPrompt("Next: cliff test, after the press cover the sensor or lift the robot");

    if (readValue(value) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::NO_RESPONSE);
        return false;
    }

    low  = value;
    high = value;

    gLogger.info("Cliff measuring, baseline:%u", value);

    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < CLIFF_WINDOW_MS)
    {
        readValue(value);

        if (value < low)
        {
            low = value;
        }

        if (value > high)
        {
            high = value;
        }

        HAL_Delay(CLIFF_SAMPLE_MS);
    }

    uint16_t span = uint16_t(high - low);
    gLogger.info("Cliff low:%u high:%u span:%u", low, high, span);

    if (span < CLIFF_CHANGE_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::CLIFF, 0, Fault::NO_CHANGE);
        return false;
    }

    out_status = makeOk(gThisBoard, ComponentId::CLIFF, 0);

    return true;
}
