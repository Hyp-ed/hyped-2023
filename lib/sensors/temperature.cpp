#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::core::ILogger &log, io::I2c &i2c, const std::uint8_t channel)
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
  // Status checks
  if (status_check_result.value() == kBusy) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read, temperature is below lower limit at channel %d",
                channel_);
    return std::nullopt;
  }
  if (status_check_result.value() == kOverTemperatureHighLimit) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read, temperature is above higher limit at channel %d",
                channel_);
    return std::nullopt;
  }
  if (status_check_result.value() == kUnderTemperatureLowLimit) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to read, temperature sensor is not ready to be read at channel %d",
                channel_);
    return std::nullopt;
  }
  // Reading the temperature and making checks if it retrieved the values
  const auto temperature_high_byte = i2c_.readByte(kTemperatureDefaultAddress, kDataTH);
  if (!temperature_high_byte) {
    logger_.log(
      hyped::core::LogLevel::kFatal, "Failed to read temperature high at channel %d", channel_);
    return std::nullopt;
  }
  const auto temperature_low_byte = i2c_.readByte(kTemperatureDefaultAddress, kDataTL);
  if (!temperature_low_byte) {
    logger_.log(
      hyped::core::LogLevel::kFatal, "Failed to read temperature low at channel %d", channel_);
    return std::nullopt;
  }
  const auto temperature = ((temperature_high_byte.value() << 8) | temperature_low_byte.value());

  logger_.log(
    hyped::core::LogLevel::kFatal, "Successfully read temperature sensor at channel %d", channel_);
  // Scaling temperature as per the datasheet
  return temperature * 0.01;
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
  logger_.log(hyped::core::LogLevel::kFatal,
              "Successful to configure temperature sensor at channel %d",
              channel_);
  return core::Result::kSuccess;
}

std::uint8_t Temperature::getChannel()
{
  return channel_;
}

}  // namespace hyped::sensors