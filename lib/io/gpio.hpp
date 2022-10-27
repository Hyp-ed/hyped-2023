#pragma once

#include "gpio_reader.hpp"
#include "gpio_writer.hpp"

#include <cstdint>
#include <optional>

#include <core/logger.hpp>

namespace hyped::io {

class Gpio {
 public:
  Gpio(hyped::core::ILogger &log);

  std::optional<GpioReader> getReader(const std::uint8_t pin);
  std::optional<GpioWriter> getWriter(const std::uint8_t pin);

 private:
  hyped::core::ILogger &log_;
};

}  // namespace hyped::io