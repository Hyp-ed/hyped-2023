#pragma once

#include "gpio.hpp"

#include <core/types.hpp>

namespace hyped::io {

class GpioReader : public IGpioReader {
 public:
  GpioReader();

  virtual core::DigitalSignal read();
};

class GpioWriter : public IGpioWriter {
 public:
  GpioWriter(const uint8_t pin);

  virtual void write(const core::DigitalSignal state);
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
