#pragma once

#include <core/types.hpp>

namespace hyped::io {

class IGpioReader {
 public:
  virtual core::DigitalSignal read() = 0;
};

class GpioReader : public IGpioReader {
 public:
  GpioReader();
  core::DigitalSignal read();
};

}  // namespace hyped::io
