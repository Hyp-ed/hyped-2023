#include "gyroscope.hpp"

namespace hyped::sensors {

Gyroscope::Gyroscope(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

Gyroscope::~Gyroscope()
{
}

std::optional<std::int16_t> Gyroscope::read(core::GyroscopeAxis axis)
{
  if (axis == core::GyroscopeAxis::kX) {
    const auto gyroscope_x_high_byte = i2c_.readByte(kGyroscope, kDataXHigh);
    if (!gyroscope_x_high_byte) {
      logger_.log(hyped::core::LogLevel::kFatal,
                  "Failed to, read gyroscope X-axis high at channel %d",
                  channel_);
      return std::nullopt;
    }
    const auto gyroscope_x_low_byte = i2c_.readByte(kGyroscope, kDataXLow);
    if (!gyroscope_x_low_byte) {
      logger_.log(hyped::core::LogLevel::kFatal,
                  "Failed to, read gyroscope X-axis low at channel %d",
                  channel_);
      return std::nullopt;
    }
    const auto x_axis = ((gyroscope_x_high_byte.value() << 8) | gyroscope_x_low_byte.value());
    logger_.log(hyped::core::LogLevel::kDebug,
                "Successfully read x-axis from gyroscope at channel %d",
                channel_);
    return x_axis;
  } else if (axis == core::GyroscopeAxis::kY) {
    const auto gyroscope_y_high_byte = i2c_.readByte(kGyroscope, kDataYHigh);
    if (!gyroscope_y_high_byte) {
      logger_.log(hyped::core::LogLevel::kFatal,
                  "Failed to, read gyroscope Y-axis high at channel %d",
                  channel_);
      return std::nullopt;
    }
    const auto gyroscope_y_low_byte = i2c_.readByte(kGyroscope, kDataYLow);
    if (!gyroscope_y_low_byte) {
      logger_.log(hyped::core::LogLevel::kFatal,
                  "Failed to, read gyroscope Y-axis low at channel %d",
                  channel_);
      return std::nullopt;
    }
    const auto y_axis = ((gyroscope_y_high_byte.value() << 8) | gyroscope_y_low_byte.value());
    logger_.log(hyped::core::LogLevel::kDebug,
                "Successfully read y-axis from gyroscope at channel %d",
                channel_);
    return y_axis;
  } else if (axis == core::GyroscopeAxis::kZ) {
    const auto gyroscope_z_high_byte = i2c_.readByte(kGyroscope, kDataZHigh);
    if (!gyroscope_z_high_byte) {
      logger_.log(hyped::core::LogLevel::kFatal,
                  "Failed to, read gyroscope Z-axis high at channel %d",
                  channel_);
      return std::nullopt;
    }
    const auto gyroscope_z_low_byte = i2c_.readByte(kGyroscope, kDataZLow);
    if (!gyroscope_z_low_byte) {
      logger_.log(hyped::core::LogLevel::kFatal,
                  "Failed to, read gyroscope Z-axis low at channel %d",
                  channel_);
      return std::nullopt;
    }
    const auto z_axis = ((gyroscope_z_high_byte.value() << 8) | gyroscope_z_low_byte.value());
    logger_.log(hyped::core::LogLevel::kDebug,
                "Successfully read z-axis from gyroscope at channel %d",
                channel_);
    return z_axis;
  } else {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to, read gyroscope at channel %d, axis parameters are invalid types",
                channel_);
    return std::nullopt;
  }
}

std::optional<Gyroscope> Gyroscope::create(core::ILogger &logger,
                                           io::II2c &i2c,
                                           const std::uint8_t channel)
{
  const core::Result write_result1
    = i2c.writeByteToRegister(kGyroscope, kCtrl1, kConfigurationSetting1);
  if (write_result1 == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal,
               "Failed to, configure gyroscope control 1 at channel %d",
               channel);
    return std::nullopt;
  }
  const core::Result write_result2
    = i2c.writeByteToRegister(kGyroscope, kCtrl2, kConfigurationSetting2);
  if (write_result2 == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal,
               "Failed to, configure gyroscope control 2 at channel %d",
               channel);
    return std::nullopt;
  }
  const core::Result write_result3
    = i2c.writeByteToRegister(kGyroscope, kCtrl3, kConfigurationSetting3);
  if (write_result3 == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal,
               "Failed to, configure gyroscope control 3 at channel %d",
               channel);
    return std::nullopt;
  }
  const core::Result write_result5
    = i2c.writeByteToRegister(kGyroscope, kCtrl5, kConfigurationSetting5);
  if (write_result5 == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal,
               "Failed to, configure gyroscope control 5 at channel %d",
               channel);
    return std::nullopt;
  }
  return Gyroscope(logger, i2c, channel);
}

core::Result Gyroscope::configure()
{
  const core::Result write_result1
    = i2c_.writeByteToRegister(kGyroscope, kCtrl1, kConfigurationSetting1);
  if (write_result1 == hyped::core::Result::kFailure) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to, configure gyroscope control 1 at channel %d",
                channel_);
    return core::Result::kFailure;
  }
  const core::Result write_result2
    = i2c_.writeByteToRegister(kGyroscope, kCtrl2, kConfigurationSetting2);
  if (write_result2 == hyped::core::Result::kFailure) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to, configure gyroscope control 2 at channel %d",
                channel_);
    return core::Result::kFailure;
  }
  const core::Result write_result3
    = i2c_.writeByteToRegister(kGyroscope, kCtrl3, kConfigurationSetting3);
  if (write_result3 == hyped::core::Result::kFailure) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to, configure gyroscope control 3 at channel %d",
                channel_);
    return core::Result::kFailure;
  }
  const core::Result write_result5
    = i2c_.writeByteToRegister(kGyroscope, kCtrl5, kConfigurationSetting5);
  if (write_result5 == hyped::core::Result::kFailure) {
    logger_.log(hyped::core::LogLevel::kFatal,
                "Failed to, configure gyroscope control 5 at channel %d",
                channel_);
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

std::uint8_t Gyroscope::getChannel()
{
  return channel_;
}

}  // namespace hyped::sensors