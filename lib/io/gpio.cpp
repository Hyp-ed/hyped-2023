#include "gpio.hpp"

namespace hyped::io {

Gpio::Gpio()
{
  // TODO: implement
}

std::optional<GpioReader> Gpio::getReader(const uint8_t pin)
{
  // TODO: implement
  return std::nullopt;
}

std::optional<GpioWriter> Gpio::getWriter(const uint8_t pin)
{
  // TODO: implement
  return std::nullopt;
}

}  // namespace hyped::io