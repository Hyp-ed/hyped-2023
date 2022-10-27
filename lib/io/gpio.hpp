#pragma once

#include "gpio_reader.hpp"
#include "gpio_writer.hpp"

#include <cstdint>
#include <optional>

#include <core/logger.hpp>

namespace hyped::io {

/**
 * An abstract GPIO interface. This is to be used in all places where GPIO access is
 * required. It is the callees responsibility to provide a correct implementation
 * such as `Gpio` below.
 */
class IGpio {
 public:
  virtual std::optional<IGpioReader> getReader(const std::uint8_t pin) = 0;
  virtual std::optional<IGpioWriter> getWriter(const std::uint8_t pin) = 0;
};

class Gpio {
 public:
  Gpio(hyped::core::ILogger &log);

  std::optional<GpioReader> getReader(const std::uint8_t pin);
  std::optional<GpioWriter> getWriter(const std::uint8_t pin);

 private:
  hyped::core::ILogger &log_;
};

}  // namespace hyped::io
