# Mo-chan STM32 self-test framework

A class-based test/boot framework for the Motion Controller (STM32), designed to
drop into a CubeMX + VS Code (CMake) project alongside the generated C code.

## File map

    app/
      status_codes.h        SHARED with ESP32 - packed StatusCode vocabulary
      status_codes.cpp      debug-only decode strings (DEBUG_BUILD)
      link_protocol.h       SHARED with ESP32 - UART command/response framing
      board_config.h        component + test enable flags (this board)
      pins.h                pins & per-component params, grouped by component
      selftest.h            ISelfTest interface + IConsole IO abstraction
      app_main.h / .cpp     entry points + boot orchestration + LED/link service
      components/
        drivetrain.hpp/.cpp motors+encoders, closed-loop motion test (showcase)
        ir_array.hpp/.cpp   IR pairs: presence, lit-delta, crosstalk
        imu.hpp/.cpp        MPU6500: WHO_AM_I, accel=1g, gyro bias, axis test
        analog_sensors.hpp  Cliff + Power (Vrefint/VDDA, vbat, reset cause)

`status_codes.*` and `link_protocol.h` are meant to be copied byte-identical to
the ESP32 project so both ends share one error vocabulary.

## Wire it into the generated main.c

    /* USER CODE BEGIN Includes */
    #include "app_main.h"
    /* USER CODE END Includes */

    /* after the MX_*_Init() block, USER CODE BEGIN 2 */
    app_setup();
    /* USER CODE END 2 */

    /* inside while(1), USER CODE BEGIN 3 */
    app_loop();
    /* USER CODE END 3 */

## CMake (VS Code STM32 extension)

Enable C++ and add the sources/includes:

    enable_language(CXX)
    target_sources(${PROJECT_NAME} PRIVATE
        app/app_main.cpp
        app/status_codes.cpp
        app/components/drivetrain.cpp
        app/components/ir_array.cpp
        app/components/imu.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE app)
    # debug config only:
    target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:DEBUG_BUILD>)

Embedded C++ flags worth setting: -fno-exceptions -fno-rtti -fno-threadsafe-statics.

## Before it compiles, set these in CubeMX

GPIO **user labels** must exist (the headers reference them):
LD2, MOTOR_STBY, AIN1, AIN2, BIN1, BIN2, IMU_CS, IMU_INT, IR_TX_L, IR_TX_F,
IR_TX_R. Enable Vrefint on ADC1. Note PC5 is double-listed in pins.h for both
VBAT and IR pair 2 - pick one and move the other before building.

## Test buckets

- **Passive** (always, every boot, never moves, never prompts): runs in both
  debug and runtime builds; result is sent to the ESP and blinked on LD2.
- **Interactive / Motion** (debug only): prompts over the debug UART, spins the
  motors. Compiled out of the runtime build via TEST_INTERACTIVE / TEST_MOTION.

## Safety gates already wired

- STBY forced low in `Drivetrain::init()` and on every fault/exit.
- IR emitters fire only if `CFG_IR_HAS_RESISTORS` is 1 (keep it 0 until you have
  series resistors, or you risk a GPIO firing a bare LED).
- Runtime boot test is passive-only: it never moves the robot near a desk edge.