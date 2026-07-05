#pragma once

enum class ControllerState
{
    BOOT,
    PASSIVE_TEST,
    DEBUG_TEST,
    RUN,
    FAULT
};

#define CONTROLLER_MAX_TESTS    8
#define BUTTON_MIN_PRESS_MS     20
#define BUTTON_HOLD_DUMP_MS     600
#define HEARTBEAT_PERIOD_MS     1000
#define BOOT_BUTTON_SETTLE_MS   20
#define STATUS_TEXT_LEN         48

#define TELEMETRY_PERIOD_MS     20
#define TELEMETRY_VERSION       2
#define TELEMETRY_WALL_COUNT    4