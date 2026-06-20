// status_codes.h
// SHARED between STM32 (Motion Controller) and ESP32 (Compute Board).
// Keep this file byte-identical on both boards so the error vocabulary matches.
//
// A status is packed into a single uint16_t so it can be compared, switched on,
// stored, and sent over UART in two bytes:
//
//   bits 15..13  Board     (3)
//   bits 12..8   Component (5)
//   bits  7..5   Instance  (3)   which IR pair / which motor / etc.
//   bits  4..0   Fault     (5)
//
#pragma once
#include <cstdint>

enum class Board : uint8_t { None = 0, STM32 = 1, ESP32 = 2 };

enum class Component : uint8_t {
    None = 0, System, Power, Imu, Encoder, Motor, Drivetrain,
    IrEmitter, IrReceiver, IrPair, Cliff, UartLink, Display, Camera, Flash
};

enum class Fault : uint8_t {
    Ok = 0,
    NoResponse,     // device did not answer (SPI/I2C/UART)
    OutOfRange,     // value implausible
    StuckHigh,      // input railed high (likely shorted / unplugged)
    StuckLow,       // input railed low
    NoChange,       // expected a delta, saw none (emitter/motor did nothing)
    WrongDirection, // moved/counted opposite to command
    Timeout,        // test did not complete in time
    Underperform,   // moved, but far less than expected (weak link / bind)
    Crosstalk,      // neighbour responded when it should not have
    NotPresent,     // configured but no hardware detected
    NotTested,      // skipped (disabled, or no resistors, etc.)
    UserFail,       // a human said "no" during an interactive test
    InvalidId       // wrong chip ID
};

struct StatusCode {
    uint16_t raw;

    constexpr StatusCode() : raw(0) {}
    constexpr explicit StatusCode(uint16_t r) : raw(r) {}
    constexpr StatusCode(Board b, Component c, uint8_t inst, Fault f)
        : raw(uint16_t((uint16_t(b) << 13) |
                       (uint16_t(c) << 8)  |
                       (uint16_t(inst & 0x7) << 5) |
                       (uint16_t(f) & 0x1F))) {}

    constexpr Board     board()     const { return Board(raw >> 13); }
    constexpr Component component() const { return Component((raw >> 8) & 0x1F); }
    constexpr uint8_t   instance()  const { return uint8_t((raw >> 5) & 0x7); }
    constexpr Fault     fault()     const { return Fault(raw & 0x1F); }

    constexpr bool ok()   const { return fault() == Fault::Ok; }
    constexpr bool fail() const { return !ok(); }

    constexpr bool operator==(const StatusCode& o) const { return raw == o.raw; }
    constexpr bool operator!=(const StatusCode& o) const { return raw != o.raw; }
};

// Convenience: an OK code for a given board/component/instance.
constexpr StatusCode OK(Board b, Component c, uint8_t inst = 0) {
    return StatusCode(b, c, inst, Fault::Ok);
}

// Worst-of accumulator: keep the first failing code seen.
inline StatusCode& worst(StatusCode& acc, StatusCode next) {
    if (acc.ok() && next.fail()) acc = next;
    return acc;
}

#if defined(DEBUG_BUILD)
const char* board_str(Board);
const char* component_str(Component);
const char* fault_str(Fault);
// Fills buf with e.g. "STM32/IrPair[2]: NoChange". Debug build only.
void status_to_string(StatusCode, char* buf, uint32_t buflen);
#endif