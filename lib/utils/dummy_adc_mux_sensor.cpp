#include "dummy_adc_mux_sensor.hpp"

namespace hyped::utils {

DummyAdcMuxSensor::DummyAdcMuxSensor()
{
}

core::Result DummyAdcMuxSensor::configure()
{
  return core::Result::kSuccess;
}

std::optional<core::Float> DummyAdcMuxSensor::read()
{
  return 0;
}

std::uint8_t DummyAdcMuxSensor::getChannel() const
{
  return 0;
}

}  // namespace hyped::utils