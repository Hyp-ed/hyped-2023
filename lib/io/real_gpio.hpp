#pragma once

#include "gpio.hpp"

#include <core/types.hpp>

namespace hyped::io {

class GpioReader : public IGpioReader {
 public:
  virtual std::optional<core::DigitalSignal> read();

 private:
  GpioReader();
  friend class Gpio;
};

class GpioWriter : public IGpioWriter {
 public:
  virtual GpioWriteResult write(const core::DigitalSignal state);

 private:
  GpioWriter(const uint8_t pin);
  friend class Gpio;
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
