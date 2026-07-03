#include "imu_globals.h"
#include "imu.h"
#include "pins.h"
#include "globals.h"
#include "logger.h"
#include "debug_io.h"
#include "stm32f4xx_hal.h"
#include <cmath>
#include <cstdio>

static bool setChipSelect(bool selected)
{
    GPIO_PinState state = GPIO_PIN_SET;

    if (selected == true)
    {
        state = GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(Pins::IMU_CS.p_port, Pins::IMU_CS.pin, state);

    return true;
}

bool Imu::readRegister(uint8_t reg, uint8_t &out_value)
{
    uint8_t tx[2] = { uint8_t(reg | MPU_READ_FLAG), 0x00 };
    uint8_t rx[2] = { 0x00, 0x00 };

    setChipSelect(true);
    HAL_StatusTypeDef hal = HAL_SPI_TransmitReceive(Pins::IMU_SPI, tx, rx, 2, 10);
    setChipSelect(false);

    if (hal != HAL_OK)
    {
        return false;
    }

    out_value = rx[1];

    return true;
}

bool Imu::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value };

    setChipSelect(true);
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(Pins::IMU_SPI, tx, 2, 10);
    setChipSelect(false);

    if (hal != HAL_OK)
    {
        return false;
    }

    return true;
}

bool Imu::readBurst(uint8_t reg, uint8_t *p_buffer, uint8_t count)
{
    uint8_t cmd = uint8_t(reg | MPU_READ_FLAG);

    setChipSelect(true);
    HAL_StatusTypeDef hal_a = HAL_SPI_Transmit(Pins::IMU_SPI, &cmd, 1, 10);
    HAL_StatusTypeDef hal_b = HAL_SPI_Receive(Pins::IMU_SPI, p_buffer, count, 10);
    setChipSelect(false);

    if (hal_a != HAL_OK)
    {
        return false;
    }

    if (hal_b != HAL_OK)
    {
        return false;
    }

    return true;
}

bool Imu::initialize()
{
    uint8_t id = 0;

    setChipSelect(false);

    writeRegister(MPU_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    writeRegister(MPU_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(50);
    writeRegister(MPU_REG_GYRO_CONFIG, 0x00);
    writeRegister(MPU_REG_ACCEL_CONFIG, 0x00);
    writeRegister(MPU_REG_SMPLRT_DIV, 0x09);
    HAL_Delay(20);

    if (readWhoAmI(id) == false)
    {
        return false;
    }

    if (id != MPU_WHO_AM_I_VALUE)
    {
        return false;
    }

    return true;
}

const char *Imu::getName()
{
    return "Imu";
}

ComponentId Imu::getComponent()
{
    return ComponentId::IMU;
}

bool Imu::readWhoAmI(uint8_t &out_id)
{
    return readRegister(MPU_REG_WHO_AM_I, out_id);
}

bool Imu::readAccel(float &out_x, float &out_y, float &out_z)
{
    uint8_t raw[6] = {0};

    if (readBurst(MPU_REG_ACCEL_XOUT_H, raw, 6) == false)
    {
        return false;
    }

    int16_t x = int16_t((raw[0] << 8) | raw[1]);
    int16_t y = int16_t((raw[2] << 8) | raw[3]);
    int16_t z = int16_t((raw[4] << 8) | raw[5]);

    out_x = float(x) / MPU_ACCEL_LSB;
    out_y = float(y) / MPU_ACCEL_LSB;
    out_z = float(z) / MPU_ACCEL_LSB;

    return true;
}

bool Imu::readGyro(float &out_x, float &out_y, float &out_z)
{
    uint8_t raw[6] = {0};

    if (readBurst(MPU_REG_GYRO_XOUT_H, raw, 6) == false)
    {
        return false;
    }

    int16_t x = int16_t((raw[0] << 8) | raw[1]);
    int16_t y = int16_t((raw[2] << 8) | raw[3]);
    int16_t z = int16_t((raw[4] << 8) | raw[5]);

    out_x = (float(x) / MPU_GYRO_LSB) - m_gyroBias[0];
    out_y = (float(y) / MPU_GYRO_LSB) - m_gyroBias[1];
    out_z = (float(z) / MPU_GYRO_LSB) - m_gyroBias[2];

    return true;
}

bool Imu::readTemperature(float &out_celsius)
{
    uint8_t raw[2] = {0};

    if (readBurst(MPU_REG_TEMP_OUT_H, raw, 2) == false)
    {
        return false;
    }

    int16_t value = int16_t((raw[0] << 8) | raw[1]);
    out_celsius = (float(value) / MPU_TEMP_SENS) + MPU_TEMP_OFFSET;

    return true;
}

bool Imu::captureGyroBias()
{
    float   sum_x = 0.0f;
    float   sum_y = 0.0f;
    float   sum_z = 0.0f;
    int     sample = 0;
    uint8_t raw[6] = {0};

    for (sample = 0; sample < MPU_BIAS_SAMPLES; sample++)
    {
        if (readBurst(MPU_REG_GYRO_XOUT_H, raw, 6) == false)
        {
            return false;
        }

        sum_x += float(int16_t((raw[0] << 8) | raw[1])) / MPU_GYRO_LSB;
        sum_y += float(int16_t((raw[2] << 8) | raw[3])) / MPU_GYRO_LSB;
        sum_z += float(int16_t((raw[4] << 8) | raw[5])) / MPU_GYRO_LSB;
        HAL_Delay(2);
    }

    m_gyroBias[0] = sum_x / float(MPU_BIAS_SAMPLES);
    m_gyroBias[1] = sum_y / float(MPU_BIAS_SAMPLES);
    m_gyroBias[2] = sum_z / float(MPU_BIAS_SAMPLES);

    return true;
}

bool Imu::runPassiveTest(StatusCode &out_status)
{
    uint8_t id = 0;
    float   ax = 0.0f;
    float   ay = 0.0f;
    float   az = 0.0f;
    float   temperature = 0.0f;

    if (readWhoAmI(id) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::NO_RESPONSE);
        return false;
    }

    if (id != MPU_WHO_AM_I_VALUE)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::INVALID_ID);
        return false;
    }

    if (readAccel(ax, ay, az) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::NO_RESPONSE);
        return false;
    }

    float magnitude = std::sqrt((ax * ax) + (ay * ay) + (az * az));

    if (magnitude < MPU_ACCEL_MAG_MIN)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::OUT_OF_RANGE);
        return false;
    }

    if (magnitude > MPU_ACCEL_MAG_MAX)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::OUT_OF_RANGE);
        return false;
    }

    if (readTemperature(temperature) == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::NO_RESPONSE);
        return false;
    }

    gLogger.info("Imu whoami:0x%02X accel x:%.2f y:%.2f z:%.2f mag:%.2f temp:%.1f",
                 id, ax, ay, az, magnitude, temperature);

    captureGyroBias();

    out_status = makeOk(gThisBoard, ComponentId::IMU, 0);

    return true;
}

bool Imu::selectAxisRate(uint8_t axis, float &out_rate)
{
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    if (readGyro(gx, gy, gz) == false)
    {
        return false;
    }

    if (axis == 0)
    {
        out_rate = gx;
        return true;
    }

    if (axis == 1)
    {
        out_rate = gy;
        return true;
    }

    out_rate = gz;

    return true;
}

bool Imu::runInteractiveTest(StatusCode &out_status)
{
    uint8_t  axis = 0;
    uint32_t start = 0;
    float    rate = 0.0f;
    float    peak = 0.0f;

    if (gDebugBuild == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::IMU, 0, Fault::NOT_TESTED);
        return false;
    }

    for (axis = 0; axis < MPU_AXIS_COUNT; axis++)
    {
        char message[48] = {0};
        snprintf(message, sizeof(message), "Next: rotate IMU about axis %u", axis);
        debugPrompt(message);

        peak = 0.0f;
        start = HAL_GetTick();

        while ((HAL_GetTick() - start) < MPU_ROTATE_WINDOW_MS)
        {
            selectAxisRate(axis, rate);

            if (rate < 0.0f)
            {
                rate = -rate;
            }

            if (rate > peak)
            {
                peak = rate;
            }

            HAL_Delay(5);
        }

        gLogger.info("Imu axis:%u peak_dps:%d", axis, int(peak));

        if (peak < MPU_ROTATE_MIN_DPS)
        {
            out_status = makeStatus(gThisBoard, ComponentId::IMU, axis, Fault::NO_CHANGE);
            return false;
        }
    }

    out_status = makeOk(gThisBoard, ComponentId::IMU, 0);

    return true;
}
