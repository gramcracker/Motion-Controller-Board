#pragma once

#include "self_test.h"
#include "link_protocol.h"
#include "link_globals.h"
#include "status_codes.h"
#include "main.h"
#include <cstdint>

struct LinkFrame
{
    LinkCmd type;
    uint8_t length;
    uint8_t payload[LINK_MAX_PAYLOAD];
};

enum class LinkRxState : uint8_t
{
    WAIT_SOF,
    WAIT_TYPE,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC
};

class Link : public SelfTest
{
public:
    bool        initialize() override;
    const char *getName() override;
    ComponentId getComponent() override;

    bool runPassiveTest(StatusCode &out_status) override;

    bool poll(LinkFrame &out_frame);

    bool sendResponse(LinkResp type, const uint8_t *p_payload, uint8_t length);
    bool sendPong();
    bool sendBooted();
    bool sendAck();
    bool sendNack();
    bool sendResults(const StatusCode *p_results, int count);
    bool sendTestDone(StatusCode status);

    uint32_t msSinceLastFrame();
    void     restartRx();

private:
    bool    consumeByte(uint8_t value, LinkFrame &out_frame);
    uint8_t computeCrc(uint8_t type, uint8_t length, const uint8_t *p_payload);

    UART_HandleTypeDef *m_pUart = nullptr;
    bool                m_ready = false;

    LinkRxState m_rxState = LinkRxState::WAIT_SOF;
    uint8_t     m_rxType  = 0;
    uint8_t     m_rxLen   = 0;
    uint8_t     m_rxIdx   = 0;
    uint8_t     m_rxPayload[LINK_MAX_PAYLOAD] = {0};
    uint32_t    m_rxTail  = 0;
    uint32_t    m_lastFrameTick = 0;
};