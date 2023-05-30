#include "dummy_i2c_sensor.hpp"

namespace hyped::utils {

DummyI2cSensor::DummyI2cSensor()
{
}

std::optional<core::Result> DummyI2cSensor::configure()
{
  return std::nullopt;
}

std::optional<std::uint8_t> DummyI2cSensor::read()
{
  return 0;
}

std::uint8_t DummyI2cSensor::getChannel() const
{
  return 0;
}

}  // namespace hyped::utils
