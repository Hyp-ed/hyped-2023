#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::core::ILogger &log, io::I2c &i2c, const std::uint8_t channel)
    : log_(log),
      i2c_(i2c),
      channel_(channel)
{
}

Temperature::~Temperature(){
  
}

std::optional<std::int16_t> Temperature::read()
{
  const auto status_check_result = i2c_.readByte(kTemperature, kStatus);
  if (!status_check_result) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature could not read status");
    return std::nullopt;
  }
  if (status_check_result.value() == 0) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature sensor is not ready to be read from");
    return std::nullopt;
  }
  const auto temperature_high_byte = i2c_.readByte(kTemperature, kDataTH);
  if (!temperature_high_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature high could not be read");
    return std::nullopt;
  }
  const auto temperature_low_byte = i2c_.readByte(kTemperature, kDataTL);
  if (!temperature_low_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature low could not be read");
    return std::nullopt;
  }
  const auto temperature = ((temperature_high_byte.value() << 8) | temperature_low_byte.value());
  // Scaling temperature as per the datasheet
  return temperature * 0.01;
}

core::Result Temperature::configure()
{
  const core::Result write_result
    = i2c_.writeByteToRegister(kTemperature, kCtrl, kConfigurationSetting);
  if (write_result == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

std::uint8_t Temperature::getChannel()
{
  return channel_;
}

}  // namespace hyped::sensors