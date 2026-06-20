// components/analog_sensors.hpp
// Lightweight analog checks: the center cliff sensor, and power-rail / battery
// sanity. Header-only; these are short enough not to warrant a .cpp each.
#pragma once
#include "selftest.h"
#include "pins.h"
#include "stm32f4xx_hal.h"

namespace detail {
inline uint16_t adc1(uint32_t ch, uint32_t st = ADC_SAMPLETIME_84CYCLES) {
    ADC_ChannelConfTypeDef cfg {};
    cfg.Channel = ch; cfg.Rank = 1; cfg.SamplingTime = st;
    HAL_ADC_ConfigChannel(pins::ADC, &cfg);
    HAL_ADC_Start(pins::ADC);
    uint16_t v = 0;
    if (HAL_ADC_PollForConversion(pins::ADC, 5) == HAL_OK)
        v = (uint16_t)HAL_ADC_GetValue(pins::ADC);
    HAL_ADC_Stop(pins::ADC);
    return v;
}
} // namespace detail

// ---- Cliff (center only for now) ----------------------------------------
class Cliff : public ISelfTest {
public:
    uint16_t read() { return detail::adc1(pins::CLIFF_CH); }
    const char* name() const override { return "Cliff"; }
    Component component() const override { return Component::Cliff; }

    StatusCode selftestPassive() override {
        uint16_t v = read();
        if (v == 0 || v >= 4090)
            return StatusCode(THIS_BOARD, Component::Cliff, 0, Fault::OutOfRange);
        // On a desk you expect reflection above this; below it = edge or unplugged.
        if (v < pins::CLIFF_ON_TABLE_MIN)
            return StatusCode(THIS_BOARD, Component::Cliff, 0, Fault::OutOfRange);
        return OK(THIS_BOARD, Component::Cliff);
    }

    StatusCode selftestInteractive(IConsole& c) override {
#if TEST_INTERACTIVE
        c.logv("Cliff on table reads %d. Now hold it over an edge / cover it.\n",
               read());
        if (!c.confirm("Sensor blocked / over edge?"))
            return StatusCode(THIS_BOARD, Component::Cliff, 0, Fault::UserFail);
        uint16_t v = read();
        c.logv("  blocked reads %d\n", v);
        if (v >= pins::CLIFF_ON_TABLE_MIN)     // expected it to drop
            return StatusCode(THIS_BOARD, Component::Cliff, 0, Fault::NoChange);
        return OK(THIS_BOARD, Component::Cliff);
#else
        (void)c; return StatusCode(THIS_BOARD, Component::Cliff, 0, Fault::NotTested);
#endif
    }
};

// ---- Power: true VDDA via Vrefint, battery plausibility, reset cause ------
class Power : public ISelfTest {
public:
    float vdda_ = 3.3f;

    float measureVdda() {
        // 1.21V nominal internal reference; VDDA = 3.3 * VREFINT_CAL / raw is the
        // calibrated form. Simplified ratiometric version shown here.
        uint16_t raw = detail::adc1(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_480CYCLES);
        if (!raw) return 0.0f;
        vdda_ = 1.21f * 4095.0f / raw;
        return vdda_;
    }
    float measureVbat() {
        uint16_t raw = detail::adc1(pins::VBAT_CH);
        return (raw / 4095.0f) * vdda_ * pins::VBAT_DIV_RATIO;
    }

    const char* name() const override { return "Power"; }
    Component component() const override { return Component::Power; }

    StatusCode selftestPassive() override {
        float vdda = measureVdda();
        if (vdda < 3.0f || vdda > 3.6f)
            return StatusCode(THIS_BOARD, Component::Power, 0, Fault::OutOfRange);
        float vbat = measureVbat();
        // Skip battery verdict if pack absent (USB-only bring-up reads ~0).
        if (vbat > 1.0f && (vbat < pins::VBAT_MIN_V || vbat > pins::VBAT_MAX_V))
            return StatusCode(THIS_BOARD, Component::Power, 1, Fault::OutOfRange);
        return OK(THIS_BOARD, Component::Power);
    }

    // Call once at boot, before clearing flags, to learn why we last reset.
    static const char* resetCauseStr() {
        const char* s = "unknown";
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) s = "IWDG watchdog";
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  s = "software";
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))  s = "power-on";
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  s = "NRST pin";
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  s = "brown-out";
        __HAL_RCC_CLEAR_RESET_FLAGS();
        return s;
    }
};