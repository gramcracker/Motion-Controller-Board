#include "power_globals.h"
#include "power.h"
#include "adc_util.h"
#include "pins.h"
#include "globals.h"
#include "logger.h"
#include "stm32g4xx_hal.h"

static bool getResetCause(const char *&out_name)
{
    out_name = "unknown";

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
    {
        out_name = "watchdog";
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U)
    {
        out_name = "software";
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U)
    {
        out_name = "power-on";
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U)
    {
        out_name = "pin";
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();

    return true;
}

bool Power::initialize()
{
    const char *p_cause = "unknown";

    getResetCause(p_cause);
    gLogger.info("Power reset cause:%s", p_cause);

    return true;
}

const char *Power::getName()
{
    return "Power";
}

ComponentId Power::getComponent()
{
    return ComponentId::POWER;
}

bool Power::measureVdda(float &out_vdda)
{
    uint16_t raw = 0;

    if (adcReadChannel(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_480CYCLES, raw) == false)
    {
        return false;
    }

    if (raw == 0)
    {
        return false;
    }

    m_vdda   = POWER_VREF_VOLTS * POWER_ADC_FULL_SCALE / float(raw);
    out_vdda = m_vdda;

    return true;
}

bool Power::measureVbat(float &out_vbat)
{
    uint16_t raw = 0;

    if (adcReadChannel(Pins::VBAT_CH, ADC_SAMPLETIME_84CYCLES, raw) == false)
    {
        return false;
    }

    out_vbat = (float(raw) / POWER_ADC_FULL_SCALE) * m_vdda * POWER_VBAT_DIV_RATIO;

    return true;
}

bool Power::runPassiveTest(StatusCode &out_status)
{
    float vdda = 0.0f;
    float vbat = 0.0f;

    if (measureVdda(vdda) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VDDA, Fault::NO_RESPONSE);
        return false;
    }

    if (vdda < POWER_VDDA_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VDDA, Fault::OUT_OF_RANGE);
        return false;
    }

    if (vdda > POWER_VDDA_MAX)
    {
        out_status = makeStatus(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VDDA, Fault::OUT_OF_RANGE);
        return false;
    }

    if (measureVbat(vbat) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VBAT, Fault::NO_RESPONSE);
        return false;
    }

    gLogger.info("Power vdda:%.2f vbat:%.2f", vdda, vbat);

    if (vbat < POWER_VBAT_ABSENT)
    {
        out_status = makeOk(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VDDA);
        return true;
    }

    if (vbat < POWER_VBAT_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VBAT, Fault::OUT_OF_RANGE);
        return false;
    }

    if (vbat > POWER_VBAT_MAX)
    {
        out_status = makeStatus(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VBAT, Fault::OUT_OF_RANGE);
        return false;
    }

    out_status = makeOk(gThisBoard, ComponentId::POWER, POWER_INSTANCE_VDDA);

    return true;
}
