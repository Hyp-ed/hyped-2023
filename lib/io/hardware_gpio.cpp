#include "hardware_gpio.hpp"

namespace hyped::io {

std::optional<core::DigitalSignal> HardwareGpioReader::read()
{
  // TODO: implement
  throw -1;
}

GpioWriteResult HardwareGpioWriter::write(const core::DigitalSignal state)
{
  // TODO: implement
  throw -1;
}

HardwareGpio::HardwareGpio(hyped::core::ILogger &log) : log_(log)
{
  // TODO: implement
}

std::optional<std::shared_ptr<IGpioReader>> HardwareGpio::getReader(const uint8_t pin)
{
  // TODO: implement
  log_.log(hyped::core::LogLevel::kFatal, "GPIO reader not implemented");
  return std::nullopt;
}

std::optional<std::shared_ptr<IGpioWriter>> HardwareGpio::getWriter(const uint8_t pin)
{
  // TODO: implement
  log_.log(hyped::core::LogLevel::kFatal, "GPIO writer not implemented");
  return std::nullopt;
}

}  // namespace hyped::io
