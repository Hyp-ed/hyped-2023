#pragma once

#include <stdint.h>

namespace hyped::core {

/**
 * @brief Nanoseconds since epoch
 */
using Time = uint64_t;

enum class LowOrHigh { kLow = 0, kHigh };

}  // namespace hyped::core
