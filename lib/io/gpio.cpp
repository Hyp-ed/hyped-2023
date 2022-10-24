#include "gpio.hpp"

namespace hyped::io {

Gpio::Gpio(hyped::utils::ILogger &log) : log_(log)
{
  // TODO: implement
}

std::optional<GpioReader> Gpio::getReader(const uint8_t pin)
{
  // TODO: implement
  log_.fatal("GPIO reader not implemented");
  return std::nullopt;
}

std::optional<GpioWriter> Gpio::getWriter(const uint8_t pin)
{
  // TODO: implement
  log_.fatal("GPIO writer not implemented");
  return std::nullopt;
}

}  // namespace hyped::io