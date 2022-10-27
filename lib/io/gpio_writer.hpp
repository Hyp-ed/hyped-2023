#pragma once

#include <cstdint>

#include <core/types.hpp>

namespace hyped::io {

class IGpioWriter {
 public:
  virtual void write(const core::DigitalSignal state) = 0;
};

class GpioWriter : public IGpioWriter {
 public:
  GpioWriter(const uint8_t pin);

  void write(const core::DigitalSignal state);
};

}  // namespace hyped::io
