#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>  // for atoi
#include <optional>

#include <core/logger.hpp>

namespace hyped::io {

class Adc {
 public:
  /**
   * @param pin is one of the 6 analogue input pins on the bbb
   */
  Adc(const std::uint8_t pin, core::ILogger &logger);
  ~Adc();

  /**
   * @brief reads AIN value from file system
   *
   * @return std::uint16_t return two bytes for [0,4095] range
   *         because the BBB has 12-bit ADCs (2^12 = 4096)
   */
  std::optional<std::uint16_t> readValue();

  /**
   * @param    file_descriptor specifying the file voltage values are read from
   * @return   std::uint16_t returns two bytes of current voltage data
   */
  std::optional<std::uint16_t> resetAndRead4(const int file_descriptor);

 private:
  core::ILogger &logger_;
  std::uint8_t pin_;
  int file_;
};

}  // namespace hyped::io
