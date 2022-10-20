#include "gpio_reader.hpp"

namespace hyped::io {


  GpioReader::GpioReader(const std::uint8_t pin, Gpio* state) : pin_(pin), io_(state)  {

  }

  core::LowOrHigh GpioReader::read()
{
  *io_.ex
}

}  // namespace hyped::io
