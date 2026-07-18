#include "link.h"
#include "config.h"
#include "pins.h"
#include "globals.h"
#include "logger.h"
#include "stm32g4xx_hal.h"

static uint8_t s_rxDma[LINK_RX_DMA_LEN] = {0};
static Link   *s_pActiveLink = nullptr;

bool Link::initialize()
{
    m_pUart = Pins::LINK_UART;

    if (m_pUart == nullptr)
    {
        gLogger.error("Link init FAILED no uart");
        return false;
    }

    gLogger.info("Link init baud:%lu ...", (unsigned long)LINK_BAUD);

    m_rxState = LinkRxState::WAIT_SOF;
    m_rxIdx   = 0;
    m_rxLen   = 0;

    restartRx();

    m_ready       = true;
    s_pActiveLink = this;

    gLogger.info("Link init ok");

    return true;
}

const char *Link::getName()
{
    return "UART Link";
}

ComponentId Link::getComponent()
{
    return ComponentId::UART_LINK;
}

bool Link::runPassiveTest(StatusCode &out_status)
{
    if (m_ready == false)
    {
        out_status = makeStatus(gThisBoard, ComponentId::UART_LINK, 0, Fault::NO_RESPONSE);
        return false;
    }

    if (m_pUart == nullptr)
    {
        out_status = makeStatus(gThisBoard, ComponentId::UART_LINK, 0, Fault::NO_RESPONSE);
        return false;
    }

    if (HAL_UART_GetState(m_pUart) == HAL_UART_STATE_RESET)
    {
        out_status = makeStatus(gThisBoard, ComponentId::UART_LINK, 0, Fault::NO_RESPONSE);
        return false;
    }

    out_status = makeOk(gThisBoard, ComponentId::UART_LINK, 0);

    return true;
}

void Link::restartRx()
{
    if (m_pUart == nullptr)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(m_pUart);

    if (HAL_UART_Receive_DMA(m_pUart, s_rxDma, LINK_RX_DMA_LEN) != HAL_OK)
    {
        HAL_UART_AbortReceive(m_pUart);
        HAL_UART_Receive_DMA(m_pUart, s_rxDma, LINK_RX_DMA_LEN);
    }

    m_rxState = LinkRxState::WAIT_SOF;
    m_rxTail  = LINK_RX_DMA_LEN - __HAL_DMA_GET_COUNTER(m_pUart->hdmarx);
}

uint8_t Link::computeCrc(uint8_t type, uint8_t length, const uint8_t *p_payload)
{
    uint8_t buffer[2 + LINK_MAX_PAYLOAD] = {0};
    uint8_t i = 0;

    buffer[0] = type;
    buffer[1] = length;

    for (i = 0; i < length; i++)
    {
        buffer[2 + i] = p_payload[i];
    }

    return link_crc8(buffer, uint32_t(length + 2));
}

bool Link::consumeByte(uint8_t value, LinkFrame &out_frame)
{
    if (m_rxState == LinkRxState::WAIT_SOF)
    {
        if (value == LINK_SOF)
        {
            m_rxState = LinkRxState::WAIT_TYPE;
        }

        return false;
    }

    if (m_rxState == LinkRxState::WAIT_TYPE)
    {
        m_rxType  = value;
        m_rxState = LinkRxState::WAIT_LEN;

        return false;
    }

    if (m_rxState == LinkRxState::WAIT_LEN)
    {
        if (value > LINK_MAX_PAYLOAD)
        {
            m_rxState = LinkRxState::WAIT_SOF;
            return false;
        }

        m_rxLen = value;
        m_rxIdx = 0;

        if (m_rxLen == 0)
        {
            m_rxState = LinkRxState::WAIT_CRC;
        }
        else
        {
            m_rxState = LinkRxState::WAIT_PAYLOAD;
        }

        return false;
    }

    if (m_rxState == LinkRxState::WAIT_PAYLOAD)
    {
        m_rxPayload[m_rxIdx] = value;
        m_rxIdx++;

        if (m_rxIdx == m_rxLen)
        {
            m_rxState = LinkRxState::WAIT_CRC;
        }

        return false;
    }

    if (m_rxState == LinkRxState::WAIT_CRC)
    {
        uint8_t expected = computeCrc(m_rxType, m_rxLen, m_rxPayload);
        uint8_t i        = 0;

        m_rxState = LinkRxState::WAIT_SOF;

        if (value != expected)
        {
            return false;
        }

        out_frame.type   = LinkCmd(m_rxType);
        out_frame.length = m_rxLen;

        for (i = 0; i < m_rxLen; i++)
        {
            out_frame.payload[i] = m_rxPayload[i];
        }

        return true;
    }

    m_rxState = LinkRxState::WAIT_SOF;

    return false;
}

bool Link::poll(LinkFrame &out_frame)
{
    if (m_ready == false)
    {
        return false;
    }

    uint32_t remaining = __HAL_DMA_GET_COUNTER(m_pUart->hdmarx);
    uint32_t head      = LINK_RX_DMA_LEN - remaining;

    while (m_rxTail != head)
    {
        uint8_t value = s_rxDma[m_rxTail];

        m_rxTail = (m_rxTail + 1) % LINK_RX_DMA_LEN;

        if (consumeByte(value, out_frame) == true)
        {
            m_lastFrameTick = HAL_GetTick();
            return true;
        }
    }

    return false;
}

bool Link::sendResponse(LinkResp type, const uint8_t *p_payload, uint8_t length)
{
    uint8_t frame[4 + LINK_MAX_PAYLOAD] = {0};
    uint8_t i = 0;

    if (m_ready == false)
    {
        return false;
    }

    if (length > LINK_MAX_PAYLOAD)
    {
        return false;
    }

    frame[0] = LINK_SOF;
    frame[1] = uint8_t(type);
    frame[2] = length;

    for (i = 0; i < length; i++)
    {
        frame[3 + i] = p_payload[i];
    }

    frame[3 + length] = computeCrc(uint8_t(type), length, p_payload);

    if (HAL_UART_Transmit(m_pUart, frame, uint16_t(length + 4), LINK_TX_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool Link::sendPong()
{
    return sendResponse(LinkResp::Pong, nullptr, 0);
}

bool Link::sendBooted()
{
    return sendResponse(LinkResp::Booted, nullptr, 0);
}

bool Link::sendAck()
{
    return sendResponse(LinkResp::Ack, nullptr, 0);
}

bool Link::sendNack()
{
    return sendResponse(LinkResp::Nack, nullptr, 0);
}

bool Link::sendResults(const StatusCode *p_results, int count)
{
    uint8_t payload[LINK_MAX_PAYLOAD] = {0};
    int     i = 0;

    if (p_results == nullptr)
    {
        return false;
    }

    if (count < 0)
    {
        return false;
    }

    if (uint32_t(count * 2) > LINK_MAX_PAYLOAD)
    {
        return false;
    }

    for (i = 0; i < count; i++)
    {
        uint16_t raw = p_results[i].getRaw();

        payload[(i * 2) + 0] = uint8_t(raw & 0xFF);
        payload[(i * 2) + 1] = uint8_t((raw >> 8) & 0xFF);
    }

    return sendResponse(LinkResp::Results, payload, uint8_t(count * 2));
}

bool Link::sendTestDone(StatusCode status)
{
    uint8_t  payload[2] = {0};
    uint16_t raw = status.getRaw();

    payload[0] = uint8_t(raw & 0xFF);
    payload[1] = uint8_t((raw >> 8) & 0xFF);

    return sendResponse(LinkResp::TestDone, payload, 2);
}

uint32_t Link::msSinceLastFrame()
{
    return HAL_GetTick() - m_lastFrameTick;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *p_uart)
{
    if (s_pActiveLink == nullptr)
    {
        return;
    }

    if (p_uart->Instance != USART1)
    {
        return;
    }

    s_pActiveLink->restartRx();
}