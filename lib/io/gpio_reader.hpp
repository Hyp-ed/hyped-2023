#pragma once

#include <core/types.hpp>

namespace hyped::io {

class GpioReader {
 public:
  GpioReader();
  core::LowOrHigh read();
};

}  // namespace hyped::io
