// selftest.h
// Component self-test interface plus an IO abstraction so the SAME test code
// runs in debug (prompts + logs over the console) and runtime (silent, passive
// only). Each component implements selftestPassive(); motion/interactive tests
// are optional and compiled out of the runtime build.
#pragma once
#include "status_codes.h"
#include "board_config.h"

// Abstracts "ask the human" and "log a line". Debug build wires this to the
// debug UART; runtime uses a null implementation that auto-passes and no-ops.
class IConsole {
public:
    virtual void log(const char* msg) = 0;
    virtual void logv(const char* fmt, ...) = 0;
    // Returns true for "yes". Runtime null-console returns false so that any
    // test which actually requires confirmation is reported NotTested, never
    // silently passed.
    virtual bool confirm(const char* prompt) = 0;
    virtual ~IConsole() {}
};

class ISelfTest {
public:
    virtual const char* name() const = 0;
    virtual Component    component() const = 0;

    // Always-on, never moves, never prompts. Runs at every boot.
    virtual StatusCode selftestPassive() = 0;

    // Optional richer tests. Default: not implemented -> NotTested.
    virtual StatusCode selftestInteractive(IConsole&) {
        return StatusCode(THIS_BOARD, component(), 0, Fault::NotTested);
    }

    virtual ~ISelfTest() {}
};