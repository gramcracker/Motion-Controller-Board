// link_protocol.h
// SHARED. Defines the framed messages exchanged over the STM32<->ESP32 UART.
//
// Frame:  [SOF=0xA5][type:1][len:1][payload:len][crc8:1]
// Keep tiny and self-synchronising; the receiver hunts for SOF and validates crc.
#pragma once
#include <cstdint>
#include "status_codes.h"

constexpr uint8_t LINK_SOF = 0xA5;

// ESP32 -> STM32
enum class LinkCmd : uint8_t {
    Ping        = 0x01,  // are you alive?
    GetResults  = 0x02,  // send all stored self-test status codes
    RunTest     = 0x03,  // payload[0] = Component id; run that test now (debug)
    StartRun    = 0x10,  // leave self-test mode, begin normal operation
    Drive       = 0x11,  // payload: v:int16, w:int16 (little-endian); fire-and-forget
    SetIr       = 0x12,  // payload[0] = IR emitter mode
    Reboot      = 0x7E,  // soft reset (NVIC_SystemReset)
    EStop       = 0x7F,  // force STBY low immediately
};

// STM32 -> ESP32
enum class LinkResp : uint8_t {
    Pong        = 0x81,
    Results     = 0x82,  // payload = N * uint16_t status codes (little-endian)
    TestDone    = 0x83,  // payload = single uint16_t status code
    Booted      = 0x84,  // sent once, unprompted, after passive self-test
    Telemetry   = 0x85,  // periodic sensor snapshot (Phase 2; not yet emitted)
    Ack         = 0x8E,
    Nack        = 0x8F,
};

// Simple CRC-8 (poly 0x07), used both directions.
inline uint8_t link_crc8(const uint8_t* d, uint32_t n) {
    uint8_t c = 0;
    for (uint32_t i = 0; i < n; ++i) {
        c ^= d[i];
        for (int b = 0; b < 8; ++b)
            c = (c & 0x80) ? uint8_t((c << 1) ^ 0x07) : uint8_t(c << 1);
    }
    return c;
}

constexpr uint32_t LINK_MAX_PAYLOAD = 64;
constexpr uint32_t LINK_PING_TIMEOUT_MS = 1500;  // ESP waits this long for Pong