// pins.h  (STM32 / Motion Controller)
// Single source of truth for pins and per-component parameters, grouped by
// component. Discrete GPIOs use the CubeMX user-labels you set in main.h
// (e.g. MOTOR_STBY). Peripheral pins (PWM/ADC/SPI/UART) are owned by their
// HAL handles, referenced here so component code never hard-codes a handle.
#pragma once
#include "main.h"      // CubeMX: GPIO label defines (XXX_Pin / XXX_GPIO_Port)
// #include "tim.h"      // htim2 (PWM), htim3/htim4 (encoders), htim9 (loop)
// #include "adc.h"       // hadc1
// #include "spi.h"       // hspi1
#include "usart.h"     // huart1 (link), huart2 (debug console)
#include <cstdint>

struct GpioPin { GPIO_TypeDef* port; uint16_t pin; };

namespace pins {

// ---- Heartbeat / fault LED (LD2, PA5) -----------------------------------
inline const GpioPin LED_HEARTBEAT { LD2_GPIO_Port, LD2_Pin };

// ---- Motor driver (TB6612FNG) -------------------------------------------
inline const GpioPin MOTOR_STBY { MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin }; // PA8
// Direction pins, [0]=left motor (A), [1]=right motor (B)
inline const GpioPin MOTOR_IN1[2] {
    { AIN1_GPIO_Port, AIN1_Pin },   // PC10
    { BIN1_GPIO_Port, BIN1_Pin },   // PC12
};
inline const GpioPin MOTOR_IN2[2] {
    { AIN2_GPIO_Port, AIN2_Pin },   // PC11
    { BIN2_GPIO_Port, BIN2_Pin },   // PD2
};
inline TIM_HandleTypeDef* const MOTOR_PWM_TIM = &htim2;
inline const uint32_t MOTOR_PWM_CH[2] { TIM_CHANNEL_1, TIM_CHANNEL_2 }; // PA0,PA1
inline const uint32_t MOTOR_PWM_MAX = 4199;  // TIM2 ARR (20 kHz @ 84 MHz)

// ---- Encoders ------------------------------------------------------------
// [0]=left (TIM3, PA6/PA7), [1]=right (TIM4, PB6/PB7)
inline TIM_HandleTypeDef* const ENC_TIM[2] { &htim3, &htim4 };

// ---- IMU (MPU6500, SPI1) -------------------------------------------------
inline SPI_HandleTypeDef* const IMU_SPI = &hspi1;
inline const GpioPin IMU_CS  { IMU_CS_GPIO_Port,  IMU_CS_Pin  };   // PB12
inline const GpioPin IMU_INT { IMU_INT_GPIO_Port, IMU_INT_Pin };   // PB13

// ---- IR array ------------------------------------------------------------
inline ADC_HandleTypeDef* const ADC = &hadc1;
// Receiver ADC channels, indexed by IR pair (left, front, right).
inline const uint32_t IR_RX_CH[3] {
    ADC_CHANNEL_13,   // PC3
    ADC_CHANNEL_14,   // PC4  (front; wire your front receiver here)
    ADC_CHANNEL_10,   // PC0
};
// Emitter drive GPIOs, indexed by IR pair.
inline const GpioPin IR_TX[3] {
    { IR_TX_L_GPIO_Port, IR_TX_L_Pin },   // PB0
    { IR_TX_F_GPIO_Port, IR_TX_F_Pin },   // PB8
    { IR_TX_R_GPIO_Port, IR_TX_R_Pin },   // PB1
};
inline const uint32_t IR_SETTLE_US   = 50;     // emitter rise + sensor settle
inline const uint16_t IR_AMBIENT_MIN = 5;      // below -> stuck low / unplugged
inline const uint16_t IR_AMBIENT_MAX = 4000;   // above -> railed / shorted
inline const uint16_t IR_DELTA_MIN   = 80;     // lit must exceed ambient by this

// ---- Cliff (center only for now) ----------------------------------------
inline const uint32_t CLIFF_CH       = ADC_CHANNEL_11;  // PC1
inline const uint16_t CLIFF_ON_TABLE_MIN = 200;   // calibrate against your desk

// ---- Power ---------------------------------------------------------------
inline const uint32_t VBAT_CH        = ADC_CHANNEL_15;  // PC5 (1S divider)
inline const float    VBAT_DIV_RATIO = 2.0f;            // set to your divider
inline const float    VBAT_MIN_V     = 3.0f;
inline const float    VBAT_MAX_V     = 4.3f;            // 1S; >this => wrong pack

// ---- Link / debug --------------------------------------------------------
inline UART_HandleTypeDef* const LINK_UART  = &huart1;  // to ESP32
inline UART_HandleTypeDef* const DEBUG_UART = &huart2;  // to ST-Link VCP

} // namespace pins