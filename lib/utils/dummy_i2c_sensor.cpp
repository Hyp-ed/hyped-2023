#include "dummy_i2c_sensor.hpp"

namespace hyped::utils {

DummyI2cSensor::DummyI2cSensor(core::ITimeSource &time_source) : time_source_(time_source)
{
}

core::Result DummyI2cSensor::configure()
{
  return core::Result::kSuccess;
}

std::optional<core::Measurement<std::uint8_t>> DummyI2cSensor::read()
{
  return core::Measurement<std::uint8_t>(time_source_.now(), 0);
}

std::uint8_t DummyI2cSensor::getChannel() const
{
  return 0;
}

}  // namespace hyped::utils
