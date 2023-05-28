#include "wheel_encoder.hpp"

namespace hyped::sensors {

WheelEncoder::WheelEncoder(core::ILogger &logger, std::shared_ptr<io::IAdc> adc)
    : logger_(logger),
      adc_(adc),
      count_(0)
{
}

std::uint64_t WheelEncoder::getCount()
{
  return count_;
}

core::Result WheelEncoder::updateCount()
{
  const auto optional_voltage = adc_->readValue();
  if (!optional_voltage) {
    logger_.log(core::LogLevel::kFatal, "Failed to read wheel encoder value from ADC");
    return core::Result::kFailure;
  }
  const core::Float voltage = *optional_voltage;
  if (voltage > kVoltageThreshold) { count_++; }
  return core::Result::kSuccess;
}

}  // namespace hyped::sensors