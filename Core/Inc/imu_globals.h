#pragma once

#include <cstdint>

#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_TEMP_OUT_H    0x41
#define MPU_REG_GYRO_XOUT_H   0x43
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75

#define MPU_READ_FLAG         0x80
#define MPU_WHO_AM_I_VALUE    0x70

#define MPU_ACCEL_LSB         16384.0f
#define MPU_GYRO_LSB          131.0f
#define MPU_TEMP_SENS         333.87f
#define MPU_TEMP_OFFSET       21.0f

#define MPU_ACCEL_MAG_MIN     0.85f
#define MPU_ACCEL_MAG_MAX     1.15f
#define MPU_TEMP_MIN          0.0f
#define MPU_TEMP_MAX          60.0f

#define MPU_BIAS_SAMPLES      200
#define MPU_AXIS_COUNT        3
#define MPU_ROTATE_MIN_DPS    20.0f
#define MPU_ROTATE_WINDOW_MS  2000
