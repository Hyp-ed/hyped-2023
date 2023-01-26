#pragma once

#include "adc.hpp"

#include <cstdio>
#include <cstdlib>  // for atoi

#include <core/logger.hpp>

namespace hyped::io {

class Adc : public IAdc {
 public:
  /**
   * @brief Creates an Adc instance
   * @param pin is one of the 6 analogue input pins on the bbb
   */
  static std::optional<Adc> create(core::ILogger &logger, const std::uint8_t pin);
  ~Adc();

  std::optional<std::uint16_t> readValue();

 private:
  Adc(core::ILogger &logger, const int file_descriptor);
  /**
   * @param    file_descriptor specifying the file voltage values are read from
   * @return   std::uint16_t returns two bytes of current voltage data
   */
  std::optional<std::uint16_t> resetAndRead4(const int file_descriptor);

 private:
  core::ILogger &logger_;
  std::uint8_t pin_;
  const int file_descriptor_;
};

}  // namespace hyped::io
