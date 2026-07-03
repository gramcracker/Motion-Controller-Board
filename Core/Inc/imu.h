#pragma once

#include "self_test.h"
#include <cstdint>

class Imu : public SelfTest
{
public:
    bool        initialize() override;
    const char *getName() override;
    ComponentId getComponent() override;

    bool runPassiveTest(StatusCode &out_status) override;
    bool runInteractiveTest(StatusCode &out_status) override;

    bool readWhoAmI(uint8_t &out_id);
    bool readAccel(float &out_x, float &out_y, float &out_z);
    bool readGyro(float &out_x, float &out_y, float &out_z);
    bool readTemperature(float &out_celsius);

private:
    bool readRegister(uint8_t reg, uint8_t &out_value);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readBurst(uint8_t reg, uint8_t *p_buffer, uint8_t count);
    bool captureGyroBias();
    bool selectAxisRate(uint8_t axis, float &out_rate);

    float m_gyroBias[3] = {0.0f, 0.0f, 0.0f};
};
