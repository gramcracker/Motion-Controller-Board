#include "adc_util.h"
#include "pins.h"
#include "stm32f4xx_hal.h"

bool adcReadChannel(uint32_t channel, uint32_t sample_time, uint16_t &out_value)
{
    ADC_ChannelConfTypeDef config = {0};

    config.Channel      = channel;
    config.Rank         = 1;
    config.SamplingTime = sample_time;

    if (HAL_ADC_ConfigChannel(Pins::ADC1_H, &config) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_Start(Pins::ADC1_H) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_PollForConversion(Pins::ADC1_H, 5) != HAL_OK)
    {
        HAL_ADC_Stop(Pins::ADC1_H);
        return false;
    }

    out_value = uint16_t(HAL_ADC_GetValue(Pins::ADC1_H));
    HAL_ADC_Stop(Pins::ADC1_H);

    return true;
}
