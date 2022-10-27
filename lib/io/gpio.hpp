#pragma once

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

/**
 * An abstract interface to read from a GPIO pin. This is to be used whenever read access
 * to GPIO is required.
 */
class IGpioReader {
 public:
  virtual core::DigitalSignal read() = 0;
};

/**
 * An abstract interface to write to a GPIO pin. This is to be used whenever write access
 * to GPIO is required.
 */
class IGpioWriter {
 public:
  virtual void write(const core::DigitalSignal state) = 0;
};

/**
 * An abstract GPIO interface. This is to be used in all places where it is necessary to
 * initiate GPIO access. It is the callees responsibility to provide a correct implementation
 * such as `Gpio` in `real_gpio.hpp`.
 */
class IGpio {
 public:
  virtual std::optional<IGpioReader> getReader(const std::uint8_t pin) = 0;
  virtual std::optional<IGpioWriter> getWriter(const std::uint8_t pin) = 0;
};

}  // namespace hyped::io
