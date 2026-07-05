#include "app_main.h"
#include "controller.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "main.h"

void app_setup(void)
{
    gController.initialize();
    HAL_TIM_Base_Start_IT(&htim10); // Start the timer interrupt for motion control tick
}

void app_loop(void)
{
    gController.execute();
}