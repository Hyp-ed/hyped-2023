#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(io::I2c &i2c, core::ILogger &log) : log_(log), i2c_(i2c)
{
}

Accelerometer::~Accelerometer()
{
}

std::optional<float> Accelerometer::read()
{
  // TODOLater: read from i2c
  log_.log(hyped::core::LogLevel::kFatal, "Accelerometer read not implemented");
  return std::nullopt;
}

bool Accelerometer::configure()
{
  // TODOLater: configure i2c
  log_.log(hyped::core::LogLevel::kFatal, "Accelerometer configure not implemented");
  return false;
}

}  // namespace hyped::sensors