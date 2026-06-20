// components/imu.cpp
#include "imu.hpp"
#include "stm32f4xx_hal.h"
#include <cmath>

namespace { // MPU6500 registers
    constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t REG_TEMP_OUT_H   = 0x41;
    constexpr uint8_t REG_GYRO_XOUT_H  = 0x43;
    constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    constexpr uint8_t REG_WHO_AM_I     = 0x75;
    constexpr uint8_t READ = 0x80;
}

static inline void cs(bool low) {
    HAL_GPIO_WritePin(pins::IMU_CS.port, pins::IMU_CS.pin,
                      low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t Imu::rd(uint8_t reg) {
    uint8_t tx[2] = { uint8_t(reg | READ), 0x00 }, rx[2] = {0, 0};
    cs(true);
    HAL_SPI_TransmitReceive(pins::IMU_SPI, tx, rx, 2, 10);
    cs(false);
    return rx[1];
}
void Imu::wr(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg, val };
    cs(true);
    HAL_SPI_Transmit(pins::IMU_SPI, tx, 2, 10);
    cs(false);
}
void Imu::rdBurst(uint8_t reg, uint8_t* buf, uint8_t n) {
    uint8_t cmd = reg | READ;
    cs(true);
    HAL_SPI_Transmit(pins::IMU_SPI, &cmd, 1, 10);
    HAL_SPI_Receive(pins::IMU_SPI, buf, n, 10);
    cs(false);
}

uint8_t Imu::whoAmI() { return rd(REG_WHO_AM_I); }

bool Imu::init() {
    cs(false);
    wr(REG_PWR_MGMT_1, 0x80); HAL_Delay(100);   // reset
    wr(REG_PWR_MGMT_1, 0x01); HAL_Delay(50);    // clock = best
    wr(REG_GYRO_CONFIG,  0x00);                 // +/-250 dps
    wr(REG_ACCEL_CONFIG, 0x00);                 // +/-2 g
    wr(REG_SMPLRT_DIV,   0x09);                 // 100 Hz
    HAL_Delay(20);
    return whoAmI() == WHO_AM_I_EXPECTED;
}

void Imu::readAccel(float g[3]) {
    uint8_t b[6]; rdBurst(REG_ACCEL_XOUT_H, b, 6);
    for (int i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)((b[2*i] << 8) | b[2*i+1]);
        g[i] = raw / accelLSB_;
    }
}
void Imu::readGyro(float dps[3]) {
    uint8_t b[6]; rdBurst(REG_GYRO_XOUT_H, b, 6);
    for (int i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)((b[2*i] << 8) | b[2*i+1]);
        dps[i] = raw / gyroLSB_ - gBias_[i];
    }
}
float Imu::readTempC() {
    uint8_t b[2]; rdBurst(REG_TEMP_OUT_H, b, 2);
    int16_t raw = (int16_t)((b[0] << 8) | b[1]);
    return raw / 333.87f + 21.0f;
}

void Imu::captureGyroBias() {
    float sum[3] = {0, 0, 0};
    const int N = 200;
    for (int k = 0; k < N; ++k) {
        uint8_t b[6]; rdBurst(REG_GYRO_XOUT_H, b, 6);
        for (int i = 0; i < 3; ++i)
            sum[i] += (int16_t)((b[2*i] << 8) | b[2*i+1]) / gyroLSB_;
        HAL_Delay(2);
    }
    for (int i = 0; i < 3; ++i) gBias_[i] = sum[i] / N;
}

StatusCode Imu::selftestPassive() {
    if (whoAmI() != WHO_AM_I_EXPECTED) {
        // Distinguish "nothing there" (0x00/0xFF) from "wrong chip".
        uint8_t id = whoAmI();
        Fault f = (id == 0x00 || id == 0xFF) ? Fault::NoResponse : Fault::InvalidId;
        return StatusCode(THIS_BOARD, Component::Imu, 0, f);
    }
    float g[3]; readAccel(g);
    float mag = std::sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    if (mag < 0.85f || mag > 1.15f)         // must be sensing ~1g of gravity
        return StatusCode(THIS_BOARD, Component::Imu, 0, Fault::OutOfRange);

    float t = readTempC();
    if (t < 0.0f || t > 60.0f)              // garbage SPI read catch
        return StatusCode(THIS_BOARD, Component::Imu, 0, Fault::OutOfRange);

    captureGyroBias();                      // assume at rest in start cell
    return OK(THIS_BOARD, Component::Imu);
}

StatusCode Imu::selftestInteractive(IConsole& c) {
#if TEST_INTERACTIVE
    const char* axis[3] = { "X (roll)", "Y (pitch)", "Z (yaw)" };
    for (int i = 0; i < 3; ++i) {
        c.logv("Rotate the robot about %s, then it will measure...\n", axis[i]);
        if (!c.confirm("Ready?"))
            return StatusCode(THIS_BOARD, Component::Imu, i, Fault::UserFail);
        float peak = 0;
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 2000) {
            float d[3]; readGyro(d);
            float a = d[i] < 0 ? -d[i] : d[i];
            if (a > peak) peak = a;
            HAL_Delay(5);
        }
        c.logv("  peak %s rate = %d dps\n", axis[i], (int)peak);
        if (peak < 20.0f)                   // barely moved -> axis dead/swapped
            return StatusCode(THIS_BOARD, Component::Imu, i, Fault::NoChange);
    }
    return OK(THIS_BOARD, Component::Imu);
#else
    (void)c;
    return StatusCode(THIS_BOARD, Component::Imu, 0, Fault::NotTested);
#endif
}