#pragma once

#include <cstdint>
#include "main.h"

class Logger
{
public:
    bool initialize(UART_HandleTypeDef *p_uart);
    void info(const char *p_format, ...);
    void error(const char *p_format, ...);

private:
    void writeLine(const char *p_text);

    UART_HandleTypeDef *m_pUart = nullptr;
    char                m_buffer[160] = {0};
};

extern Logger gLogger;
