#pragma once

#include "adc.hpp"

#include <cstdio>
#include <cstdlib>  // for atoi
#include <memory>

#include <core/logger.hpp>

namespace hyped::io {

class Adc : public IAdc {
 public:
  /**
   * @brief Creates an Adc instance
   * @param pin is one of the 6 analogue input pins on the bbb
   */
  static std::optional<std::shared_ptr<Adc>> create(core::ILogger &logger, const std::uint8_t pin);
  Adc(core::ILogger &logger, const int file_descriptor);
  ~Adc();

  std::optional<std::uint16_t> readValue();

 private:
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
