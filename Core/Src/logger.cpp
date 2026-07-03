#include "logger.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

Logger gLogger;

bool Logger::initialize(UART_HandleTypeDef *p_uart)
{
    if (p_uart == nullptr)
    {
        return false;
    }

    m_pUart = p_uart;

    return true;
}

void Logger::writeLine(const char *p_text)
{
    if (m_pUart == nullptr)
    {
        return;
    }

    uint16_t len = uint16_t(strlen(p_text));
    HAL_UART_Transmit(m_pUart, (uint8_t *)p_text, len, 100);
    HAL_UART_Transmit(m_pUart, (uint8_t *)"\r\n", 2, 10);
}

void Logger::info(const char *p_format, ...)
{
    va_list args;
    va_start(args, p_format);
    vsnprintf(m_buffer, sizeof(m_buffer), p_format, args);
    va_end(args);

    writeLine(m_buffer);
}

void Logger::error(const char *p_format, ...)
{
    char    prefixed[160] = {0};
    va_list args;

    va_start(args, p_format);
    vsnprintf(m_buffer, sizeof(m_buffer), p_format, args);
    va_end(args);

    snprintf(prefixed, sizeof(prefixed), "ERROR: %s", m_buffer);
    writeLine(prefixed);
}
