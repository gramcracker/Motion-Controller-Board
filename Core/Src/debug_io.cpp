#include "debug_io.h"
#include "logger.h"
#include "main.h"
#include "stm32g4xx_hal.h"

bool debugPrompt(const char *p_message)
{
    gLogger.info("%s, push button to continue", p_message);

    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(5);
    }

    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
    {
        HAL_Delay(5);
    }

    HAL_Delay(20);

    return true;
}
