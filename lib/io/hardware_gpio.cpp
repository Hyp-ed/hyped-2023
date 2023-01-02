#include "hardware_gpio.hpp"

namespace hyped::io {

std::optional<core::DigitalSignal> HardwareGpioReader::readPin()
{
  // TODOLater: implement
  throw -1;
}

core::Result HardwareGpioWriter::writeToPin(const core::DigitalSignal state)
{
  // TODOLater: implement
  throw -1;
}

HardwareGpio::HardwareGpio(core::ILogger &log) : log_(log)
{
  // TODOLater: implement
}

std::optional<std::shared_ptr<IGpioReader>> HardwareGpio::getReader(const std::uint8_t pin)
{
  // TODOLater: implement
  log_.log(core::LogLevel::kFatal, "GPIO reader not implemented");
  return std::nullopt;
}

std::optional<std::shared_ptr<IGpioWriter>> HardwareGpio::getWriter(const std::uint8_t pin)
{
  // TODOLater: implement
  log_.log(core::LogLevel::kFatal, "GPIO writer not implemented");
  return std::nullopt;
}

}  // namespace hyped::io