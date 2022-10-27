#include "real_gpio.hpp"

namespace hyped::io {

std::optional<core::DigitalSignal> GpioReader::read()
{
  // TODO: implement
  throw -1;
}

GpioWriteResult GpioWriter::write(const core::DigitalSignal state)
{
  // TODO: implement
  throw -1;
}

Gpio::Gpio(hyped::core::ILogger &log) : log_(log)
{
  // TODO: implement
}

std::optional<std::shared_ptr<IGpioReader>> Gpio::getReader(const uint8_t pin)
{
  // TODO: implement
  log_.log(hyped::core::LogLevel::kFatal, "GPIO reader not implemented");
  return std::nullopt;
}

std::optional<std::shared_ptr<IGpioWriter>> Gpio::getWriter(const uint8_t pin)
{
  // TODO: implement
  log_.log(hyped::core::LogLevel::kFatal, "GPIO writer not implemented");
  return std::nullopt;
}

}  // namespace hyped::io
