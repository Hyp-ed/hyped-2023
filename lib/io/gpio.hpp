#pragma once

#include "gpio_reader.hpp"
#include "gpio_writer.hpp"

#include <cstdint>
#include <optional>

namespace hyped::io {

class Gpio {
 public:
  Gpio();

  std::optional<GpioReader> getReader(const std::uint8_t pin);
  std::optional<GpioWriter> getWriter(const std::uint8_t pin);
};

}  // namespace hyped::io