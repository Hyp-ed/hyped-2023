#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(const std::uint8_t mux_address,
                         const std::uint8_t channel,
                         hyped::io::I2c &i2c,
                         hyped::core::ILogger &log)
    : log_(log),
      i2c_(i2c),
      channel_(channel),
      mux_address_(mux_address)
{
}

Temperature::~Temperature()
{
}

std::optional<std::uint16_t> Temperature::read()
{
  const auto status_check_result = i2c_.readByte(mux_address_, kStatus);

  const auto temperature_high = i2c_.readByte(mux_address_, kData_T_H);
  const auto temperature_low  = i2c_.readByte(mux_address_, kData_T_L);

  // getting the temperature_high_byte and temperature_low_byte values from the register

  if (!status_check_result) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature could not read status");
    return std::nullopt;
  }

  if (status_check_result.value() == 0) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature sensor is not ready to be read from");
    return std::nullopt;
  }

  if (!temperature_high) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature high could not be read");
    return std::nullopt;
  }

  if (!temperature_low) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature low could not be read");
    return std::nullopt;
  }

  std::uint16_t temperature = (temperature_high.value() << 8) | temperature_low.value();

  temperature = temperature * 0.01;

  return temperature;
  // Reading from the Status register to see if the temperature is ready to be read
  // Need to have it do the correct effect and output a log of what occurred
}

core::Result Temperature::configure()
{
  const core::Result write_result
    = i2c_.writeByteToRegister(mux_address_, kCtrl, kConfigurationSetting);
  // We need to know if a begalebone has two registers set to it
  if (write_result == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
    return core::Result::kFailure;
  }
  // Writing to the Control register 0x04 and setting that to 1 for TEMPERATURE_1
  return core::Result::kSuccess;
}

std::uint8_t Temperature::getChannel()
{
  return channel_;
}

}  // namespace hyped::sensors