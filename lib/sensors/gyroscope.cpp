#include "gyroscope.hpp"

namespace hyped::sensors {

Gyroscope::Gyroscope(hyped::core::ILogger &log, io::I2c &i2c, const std::uint8_t channel)
    : log_(log),
      i2c_(i2c),
      channel_(channel)
{
}

std::optional<std::int16_t> Gyroscope::read()
{
  const auto gyroscope_x_high_byte = i2c_.readByte(kGyroscope, kDataXH);
  if (!gyroscope_x_high_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope X-axis high could not be read");
    return std::nullopt;
  }
  const auto gyroscope_x_low_byte = i2c_.readByte(kGyroscope, kDataXL);
  if (!gyroscope_x_low_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope X-axis low could not be read");
    return std::nullopt;
  }
  const auto gyroscope_y_high_byte = i2c_.readByte(kGyroscope, kDataYH);
  if (!gyroscope_y_high_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope Y-axis high could not be read");
    return std::nullopt;
  }
  const auto gyroscope_y_low_byte = i2c_.readByte(kGyroscope, kDataYL);
  if (!gyroscope_y_low_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope Y-axis low could not be read");
    return std::nullopt;
  }
  const auto gyroscope_z_high_byte = i2c_.readByte(kGyroscope, kDataZH);
  if (!gyroscope_z_high_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope Z-axis high could not be read");
    return std::nullopt;
  }
  const auto gyroscope_z_low_byte = i2c_.readByte(kGyroscope, kDataZL);
  if (!gyroscope_z_low_byte) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope Z-axis low could not be read");
    return std::nullopt;
  }

  const auto x_axis = ((gyroscope_x_high_byte.value() << 8) | gyroscope_x_low_byte.value());
  const auto y_axis = ((gyroscope_y_high_byte.value() << 8) | gyroscope_y_low_byte.value());
  const auto z_axis = ((gyroscope_z_high_byte.value() << 8) | gyroscope_z_low_byte.value());

  // TODO the conversions of the 3 axis
  return 0;
}

core::Result Gyroscope::configure()
{
  const core::Result write_result1
    = i2c_.writeByteToRegister(kGyroscope, kCtrl1, kConfigurationSetting1);
  if (write_result1 == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope control 1 configure not implemented");
    return core::Result::kFailure;
  }
  const core::Result write_result2
    = i2c_.writeByteToRegister(kGyroscope, kCtrl2, kConfigurationSetting2);
  if (write_result2 == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope control 2 configure not implemented");
    return core::Result::kFailure;
  }
  const core::Result write_result3
    = i2c_.writeByteToRegister(kGyroscope, kCtrl3, kConfigurationSetting3);
  if (write_result3 == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope control 3 configure not implemented");
    return core::Result::kFailure;
  }
  const core::Result write_result5
    = i2c_.writeByteToRegister(kGyroscope, kCtrl5, kConfigurationSetting5);
  if (write_result5 == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Gyroscope control 5 configure not implemented");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

std::uint8_t Gyroscope::getChannel()
{
  return channel_;
}

}  // namespace hyped::sensors