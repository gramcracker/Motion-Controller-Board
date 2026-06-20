// app_main.cpp
// C++ application entry. Pattern: CubeMX main.c calls app_setup()/app_loop().
//
// Boot flow on this board (Motion Controller), per the agreed plan:
//   1. run all PASSIVE self-tests autonomously (no ESP needed, never moves)
//   2. aggregate worst status, blink it on the LED as a fallback
//   3. send LinkResp::Booted to the ESP, then serve the link
//   4. (debug) when the ESP commands RunTest, run interactive/motion tests
#include "app_main.h"
#include "board_config.h"
#include "pins.h"
#include "link_protocol.h"
#include "selftest.h"
#include "components/drivetrain.hpp"
#include "components/ir_array.hpp"
#include "components/imu.hpp"
#include "components/analog_sensors.hpp"
#include "stm32f4xx_hal.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

// ---- Components (static; no heap) ---------------------------------------
static Power      g_power;
static Imu        g_imu;
static Drivetrain g_drive;
static IrArray    g_ir;
static Cliff      g_cliff;

static ISelfTest* g_tests[] = {
#if CFG_ENABLE_POWER
    &g_power,
#endif
#if CFG_ENABLE_IMU
    &g_imu,
#endif
#if CFG_ENABLE_DRIVETRAIN
    &g_drive,
#endif
#if CFG_ENABLE_IR
    &g_ir,
#endif
#if CFG_ENABLE_CLIFF
    &g_cliff,
#endif
};
static constexpr int N_TESTS = sizeof(g_tests) / sizeof(g_tests[0]);

static StatusCode g_results[N_TESTS];
static StatusCode g_boot_status;

// ---- Console: debug = UART2, runtime = null -----------------------------
class UartConsole : public IConsole {
public:
    void log(const char* m) override {
        HAL_UART_Transmit(pins::DEBUG_UART, (uint8_t*)m, strlen(m), 50);
    }
    void logv(const char* fmt, ...) override {
        char buf[128]; va_list ap; va_start(ap, fmt);
        vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
        log(buf);
    }
    bool confirm(const char* prompt) override {
        logv("%s [y/n] ", prompt);
        uint8_t ch = 0;
        while (HAL_UART_Receive(pins::DEBUG_UART, &ch, 1, HAL_MAX_DELAY) != HAL_OK) {}
        log("\n");
        return ch == 'y' || ch == 'Y';
    }
};
class NullConsole : public IConsole {
public:
    void log(const char*) override {}
    void logv(const char*, ...) override {}
    bool confirm(const char*) override { return false; }  // -> NotTested, never silent-pass
};
#if defined(DEBUG_BUILD)
static UartConsole g_consoleImpl;
#else
static NullConsole g_consoleImpl;
#endif
static IConsole& g_console = g_consoleImpl;

// ---- LED fault fallback (bottom-of-stack indicator) ----------------------
// 1 blink = alive/ok, 2 = link fail, 3+ = a self-test failed; fast = panic.
static void ledBlink(int n, uint32_t on_ms = 120, uint32_t off_ms = 200) {
    for (int i = 0; i < n; ++i) {
        HAL_GPIO_WritePin(pins::LED_HEARTBEAT.port, pins::LED_HEARTBEAT.pin, GPIO_PIN_SET);
        HAL_Delay(on_ms);
        HAL_GPIO_WritePin(pins::LED_HEARTBEAT.port, pins::LED_HEARTBEAT.pin, GPIO_PIN_RESET);
        HAL_Delay(off_ms);
    }
}

// ---- Link TX helper ------------------------------------------------------
static void linkSend(LinkResp type, const uint8_t* payload, uint8_t len) {
    uint8_t f[4 + LINK_MAX_PAYLOAD];
    f[0] = LINK_SOF; f[1] = (uint8_t)type; f[2] = len;
    if (len) memcpy(&f[3], payload, len);
    f[3 + len] = link_crc8(&f[1], len + 2);
    HAL_UART_Transmit(pins::LINK_UART, f, len + 4, 100);
}

// ---- Boot: run passive tests autonomously --------------------------------
static void runPassive() {
    g_boot_status = StatusCode();   // OK
    g_console.logv("\n--- boot self-test (reset: %s) ---\n", Power::resetCauseStr());
    for (int i = 0; i < N_TESTS; ++i) {
        g_results[i] = g_tests[i]->selftestPassive();
        worst(g_boot_status, g_results[i]);
#if defined(DEBUG_BUILD)
        char buf[48]; status_to_string(g_results[i], buf, sizeof buf);
        g_console.logv("  [%-10s] %s\n", g_tests[i]->name(), buf);
#endif
    }
    g_console.logv("--- boot status raw=0x%04X ---\n", g_boot_status.raw);
}

// ---- Interactive/motion test dispatch (debug only) -----------------------
static StatusCode runInteractive(Component which) {
    for (int i = 0; i < N_TESTS; ++i)
        if (g_tests[i]->component() == which)
            return g_tests[i]->selftestInteractive(g_console);
    return StatusCode(THIS_BOARD, which, 0, Fault::NotTested);
}

// =========================================================================
extern "C" void app_setup(void) {
    // Hardware init for components that own peripherals.
    g_drive.init();         // forces STBY low first thing
    g_ir.init();
#if CFG_ENABLE_IMU
    g_imu.init();
#endif

    runPassive();

    // Tell the ESP we are up and what we found (it owns the screen).
    linkSend(LinkResp::Booted, (uint8_t*)&g_boot_status.raw, 2);

    // Fallback indication if the ESP/screen is dead.
    ledBlink(g_boot_status.ok() ? 1 : 3);
}

extern "C" void app_loop(void) {
    static uint32_t last_seen = 0;

    // --- Service link commands from the ESP ---
    uint8_t b;
    if (HAL_UART_Receive(pins::LINK_UART, &b, 1, 0) == HAL_OK && b == LINK_SOF) {
        uint8_t hdr[2];
        if (HAL_UART_Receive(pins::LINK_UART, hdr, 2, 20) == HAL_OK) {
            LinkCmd cmd = (LinkCmd)hdr[0];
            uint8_t len = hdr[1];
            uint8_t pl[LINK_MAX_PAYLOAD], crc;
            if (len) HAL_UART_Receive(pins::LINK_UART, pl, len, 20);
            HAL_UART_Receive(pins::LINK_UART, &crc, 1, 20);
            last_seen = HAL_GetTick();

            switch (cmd) {
            case LinkCmd::Ping:
                linkSend(LinkResp::Pong, nullptr, 0);
                break;
            case LinkCmd::GetResults:
                linkSend(LinkResp::Results, (uint8_t*)g_results,
                         (uint8_t)(N_TESTS * 2));
                break;
            case LinkCmd::RunTest: {
#if TEST_INTERACTIVE || TEST_MOTION
                StatusCode r = runInteractive((Component)(len ? pl[0] : 0));
                linkSend(LinkResp::TestDone, (uint8_t*)&r.raw, 2);
#else
                StatusCode r(THIS_BOARD, Component::System, 0, Fault::NotTested);
                linkSend(LinkResp::TestDone, (uint8_t*)&r.raw, 2);
#endif
                break;
            }
            case LinkCmd::EStop:
                g_drive.enable(false);
                linkSend(LinkResp::Ack, nullptr, 0);
                break;
            case LinkCmd::StartRun:
                linkSend(LinkResp::Ack, nullptr, 0);
                // (hand control to the TIM9 loop here in normal operation)
                break;
            default:
                linkSend(LinkResp::Nack, nullptr, 0);
            }
        }
    }

    // --- Heartbeat / link-loss indication ---
    static uint32_t last_blink = 0;
    if (HAL_GetTick() - last_blink > 1000) {
        last_blink = HAL_GetTick();
        bool link_ok = last_seen && (HAL_GetTick() - last_seen < 3000);
        ledBlink(link_ok ? 1 : 2, 60, 0);
    }
}