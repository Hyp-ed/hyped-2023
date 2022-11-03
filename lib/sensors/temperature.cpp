#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log) : log_(log), i2c_(i2c)
{
}

Temperature::~Temperature()
{
}

std::optional<uint16_t> Temperature::read()
{
  // TODOLater: read from i2c
  log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
  return std::nullopt;
}

bool Temperature::configure()
{
  // TODOLater: configure i2c
  log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
  return false;
}

}  // namespace hyped::sensors