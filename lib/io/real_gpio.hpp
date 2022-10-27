#pragma once

#include "gpio.hpp"


#include <core/types.hpp>

namespace hyped::io {

class GpioReader : public IGpioReader {
 public:
  GpioReader();

  virtual std::optional<core::DigitalSignal> read();
};

class GpioWriter : public IGpioWriter {
 public:
  GpioWriter(const uint8_t pin);

  virtual GpioWriteResult write(const core::DigitalSignal state);
};

class Gpio {
 public:
  Gpio(hyped::core::ILogger &log);

  virtual std::optional<std::shared_ptr<IGpioReader>> getReader(const std::uint8_t pin);
  virtual std::optional<std::shared_ptr<IGpioWriter>> getWriter(const std::uint8_t pin);

 private:
  hyped::core::ILogger &log_;
};

}  // namespace hyped::io
