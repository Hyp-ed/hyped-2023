#pragma once

#include <core/types.hpp>

namespace hyped::io {

class GpioReader {
  public:
    GpioReader();
    core::LowOrHigh read();
  private:
    io::Gpio* io_;
    std::uint8_t pin_;
};

}  // namespace hyped::io
