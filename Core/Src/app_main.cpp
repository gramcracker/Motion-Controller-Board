#include "app_main.h"
#include "controller.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "main.h"

void app_setup(void)
{
    gController.initialize();
}

void app_loop(void)
{
    gController.execute();
}