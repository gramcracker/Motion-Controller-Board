#pragma once

#include <cstdint>
#include "status_codes.h"

#define BUILD_DEBUG 1

inline constexpr Board gThisBoard  = Board::STM32;
inline constexpr bool  gDebugBuild = (BUILD_DEBUG == 1);
