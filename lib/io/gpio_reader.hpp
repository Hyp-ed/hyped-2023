#pragma once

#include <core/types.hpp>

namespace hyped::io {

class GpioReader {
 public:
  GpioReader();
  core::DigitalSignal read();
};

}  // namespace hyped::io
