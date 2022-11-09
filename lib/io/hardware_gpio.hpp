#pragma once

#include "gpio.hpp"

#include <core/types.hpp>

namespace hyped::io {

class HardwareGpioReader : public IGpioReader {
 public:
  virtual std::optional<core::DigitalSignal> read();

 private:
  HardwareGpioReader();
  friend class HardwareGpio;
};

class HardwareGpioWriter : public IGpioWriter {
 public:
  virtual hyped::core::Result write(const core::DigitalSignal state);

 private:
  HardwareGpioWriter(const std::uint8_t pin);
  friend class HardwareGpio;
};

/**
 * Hardware GPIO interface, requires physical GPIO pins to be present. This should only
 * be instantiated at the top level and then provided to users through the IGpio interface.
 */
class HardwareGpio {
 public:
  HardwareGpio(hyped::core::ILogger &log);

  virtual std::optional<std::shared_ptr<IGpioReader>> getReader(const std::uint8_t pin);
  virtual std::optional<std::shared_ptr<IGpioWriter>> getWriter(const std::uint8_t pin);

 private:
  hyped::core::ILogger &log_;
};

}  // namespace hyped::io
