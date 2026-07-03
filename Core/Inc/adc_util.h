#pragma once

#include <cstdint>

bool adcReadChannel(uint32_t channel, uint32_t sample_time, uint16_t &out_value);
