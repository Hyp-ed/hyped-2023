#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::core::ILogger &logger,
                         io::HardwareI2c &i2c,
                         const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

Temperature::~Temperature()
{
}

std::optional<std::int16_t> Temperature::read()
{
  const auto status_check_result = i2c_.readByte(kTemperatureDefaultAddress, kStatus);
  if (!status_check_result) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read temperature sensor status at channel %d",
                channel_);
    return std::nullopt;
  }
  // TODOLater: change this from a fatal error to a debug and re-prompt read (with ROS)
  if (status_check_result.value() == kBusy) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read, temperature sensor is not ready to be read at channel %d",
                channel_);
    return std::nullopt;
  } else if (status_check_result.value() == kTemperatureOverUpperLimit) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read, temperature is above upper limit at channel %d",
                channel_);
    return std::nullopt;
  } else if (status_check_result.value() == kTemperatureUnderLowerLimit) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read, temperature is below lower limit at channel %d",
                channel_);
    return std::nullopt;
  }
  const auto temperature_high_byte
    = i2c_.readByte(kTemperatureDefaultAddress, kDataTemperatureHigh);
  if (!temperature_high_byte) {
    logger_.log(
      hyped::core::LogLevel::kFatal, "Failed to read temperature high at channel %d", channel_);
    return std::nullopt;
  }
  const auto temperature_low_byte = i2c_.readByte(kTemperatureDefaultAddress, kDataTemperatureLow);
  if (!temperature_low_byte) {
    logger_.log(
      hyped::core::LogLevel::kFatal, "Failed to read temperature low at channel %d", channel_);
    return std::nullopt;
  }
  const std::int16_t temperature
    = ((temperature_high_byte.value() << 8) | temperature_low_byte.value());
  logger_.log(
    hyped::core::LogLevel::kDebug, "Successfully read temperature sensor at channel %d", channel_);
  // Scaling temperature as per the datasheet
  return temperature * kTemperatureScaleFactor;
}

core::Result Temperature::configure()
{
  const core::Result write_result
    = i2c_.writeByteToRegister(kTemperatureDefaultAddress, kCtrl, kConfigurationSetting);
  if (write_result == hyped::core::Result::kFailure) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to configure temperature sensor at channel %d",
                channel_);
    return core::Result::kFailure;
  }
  logger_.log(hyped::core::LogLevel::kDebug,
              "Successful to configure temperature sensor at channel %d",
              channel_);
  return core::Result::kSuccess;
}

std::uint8_t Temperature::getChannel()
{
  return channel_;
}

}  // namespace hyped::sensors