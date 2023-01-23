#pragma once

#include <cstdint>
#include <optional>

namespace hyped::io {

class IAdc {
 public:
  /**
   * @brief reads AIN value
   * @return two bytes in range [0,4095] because the BBB has 12-bit ADCs (2^12 = 4096)
   */
  std::optional<std::uint16_t> readValue();
};

}  // namespace hyped::io
