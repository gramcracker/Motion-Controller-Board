#pragma once
#include "main.h"      // CubeMX: GPIO label defines (XXX_Pin / XXX_GPIO_Port)
#include "tim.h"      // htim2 (PWM), htim3/htim4 (encoders), htim6 (1 kHz loop on G4)
#include "adc.h"       // hadc2 (all analog moved to ADC2 for the 48-pin G431)
#include "spi.h"       // hspi1
#include "usart.h"     // huart1 (link), huart2 (debug console)
#include <cstdint>

struct GpioPin
{
    GPIO_TypeDef *p_port;
    uint16_t      pin;
};

namespace Pins
{
    inline const GpioPin LED_HEARTBEAT = { LD2_GPIO_Port, LD2_Pin };

    inline const GpioPin MOTOR_STBY    = { MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin };
    inline const GpioPin MOTOR_IN1[2]  = { { AIN1_GPIO_Port, AIN1_Pin },
                                           { BIN1_GPIO_Port, BIN1_Pin } };
    inline const GpioPin MOTOR_IN2[2]  = { { AIN2_GPIO_Port, AIN2_Pin },
                                           { BIN2_GPIO_Port, BIN2_Pin } };

    inline TIM_HandleTypeDef *const MOTOR_PWM_TIM   = &htim2;
    inline const uint32_t           MOTOR_PWM_CH[2] = { TIM_CHANNEL_1, TIM_CHANNEL_2 };
    inline const uint16_t           MOTOR_PWM_MAX   = 4199;

    inline TIM_HandleTypeDef *const ENC_TIM[2] = { &htim3, &htim4 };

    inline SPI_HandleTypeDef *const IMU_SPI = &hspi1;
    inline const GpioPin            IMU_CS  = { IMU_CS_GPIO_Port,  IMU_CS_Pin  };
    inline const GpioPin            IMU_INT = { IMU_INT_GPIO_Port, IMU_INT_Pin };

    inline ADC_HandleTypeDef *const ADC_H     = &hadc2;
    inline const uint32_t           CLIFF_CH  = ADC_CHANNEL_17;   // G431: PA4
    inline const uint32_t           VBAT_CH   = ADC_CHANNEL_15;   // G431: PB15

    inline UART_HandleTypeDef *const LINK_UART  = &huart1;
    inline UART_HandleTypeDef *const DEBUG_UART = &huart2;
}