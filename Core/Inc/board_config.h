// board_config.h  (STM32 / Motion Controller)
// Per-board enable flags. Flip these to include/exclude components and tests.
#pragma once
#include "status_codes.h"

#define THIS_BOARD  Board::STM32

// ---- Build flavour -------------------------------------------------------
// Define DEBUG_BUILD in your debug CMake config (target_compile_definitions).
// Runtime (release) build runs ONLY passive tests, never moves, never prompts.
#if defined(DEBUG_BUILD)
  #define TEST_INTERACTIVE 1   // y/n prompts over the debug UART
  #define TEST_MOTION      1   // tests that spin the motors
#else
  #define TEST_INTERACTIVE 0
  #define TEST_MOTION      0
#endif

// ---- Component enables ---------------------------------------------------
#define CFG_ENABLE_POWER       1
#define CFG_ENABLE_IMU         1
#define CFG_ENABLE_DRIVETRAIN  1
#define CFG_ENABLE_IR          1
#define CFG_ENABLE_CLIFF       1
#define CFG_ENABLE_LINK        1

// ---- IR specifics --------------------------------------------------------
#define CFG_IR_PAIR_COUNT      3      // left, front, right (scale up to 4 later)
#define CFG_IR_EMITTERS_ENABLED 1
// SAFETY: emitters are only fired if resistors are present. Until you have
// series resistors installed, leave this 0 and the IR test runs ambient-only.
#define CFG_IR_HAS_RESISTORS   0

// ---- Drivetrain test params ---------------------------------------------
#define CFG_MOTOR_TEST_DUTY    1500   // out of TIM2 ARR (~36% of 4199)
#define CFG_MOTOR_TEST_MS      300    // run each direction this long
#define CFG_MOTOR_MIN_COUNTS   40     // expect at least this many encoder counts
#define CFG_MOTOR_TIMEOUT_MS   600    // hard ceiling per move

// ---- Control loop --------------------------------------------------------
#define CFG_LOOP_HZ            1000