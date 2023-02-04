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

const std::optional<std::int16_t> Gyroscope::read(core::Axis axis)
{
  switch (axis) {
    case core::Axis::kX: {
      const auto gyroscope_x_high_byte = i2c_.readByte(kDefaultGyroscopeAddress, kDataXHigh);
      if (!gyroscope_x_high_byte) {
        logger_.log(
          core::LogLevel::kFatal, "Failed to read gyroscope X-axis high at channel %d", channel_);
        return std::nullopt;
      }
      const auto gyroscope_x_low_byte = i2c_.readByte(kDefaultGyroscopeAddress, kDataXLow);
      if (!gyroscope_x_low_byte) {
        logger_.log(
          core::LogLevel::kFatal, "Failed to read gyroscope X-axis low at channel %d", channel_);
        return std::nullopt;
      }
      const auto x_axis = ((gyroscope_x_high_byte.value() << 8) | gyroscope_x_low_byte.value());
      logger_.log(
        core::LogLevel::kDebug, "Successfully read x-axis from gyroscope at channel %d", channel_);
      return x_axis;
    }
    case core::Axis::kY: {
      const auto gyroscope_y_high_byte = i2c_.readByte(kDefaultGyroscopeAddress, kDataYHigh);
      if (!gyroscope_y_high_byte) {
        logger_.log(
          core::LogLevel::kFatal, "Failed to read gyroscope Y-axis high at channel %d", channel_);
        return std::nullopt;
      }
      const auto gyroscope_y_low_byte = i2c_.readByte(kDefaultGyroscopeAddress, kDataYLow);
      if (!gyroscope_y_low_byte) {
        logger_.log(
          core::LogLevel::kFatal, "Failed to read gyroscope Y-axis low at channel %d", channel_);
        return std::nullopt;
      }
      const auto y_axis = ((gyroscope_y_high_byte.value() << 8) | gyroscope_y_low_byte.value());
      logger_.log(
        core::LogLevel::kDebug, "Successfully read y-axis from gyroscope at channel %d", channel_);
      return y_axis;
    }
    case core::Axis::kZ: {
      const auto gyroscope_z_high_byte = i2c_.readByte(kDefaultGyroscopeAddress, kDataZHigh);
      if (!gyroscope_z_high_byte) {
        logger_.log(
          core::LogLevel::kFatal, "Failed to read gyroscope Z-axis high at channel %d", channel_);
        return std::nullopt;
      }
      const auto gyroscope_z_low_byte = i2c_.readByte(kDefaultGyroscopeAddress, kDataZLow);
      if (!gyroscope_z_low_byte) {
        logger_.log(
          core::LogLevel::kFatal, "Failed to read gyroscope Z-axis low at channel %d", channel_);
        return std::nullopt;
      }
      const auto z_axis = ((gyroscope_z_high_byte.value() << 8) | gyroscope_z_low_byte.value());
      logger_.log(
        core::LogLevel::kDebug, "Successfully read z-axis from gyroscope at channel %d", channel_);
      return z_axis;
    }
  }
}

std::optional<Gyroscope> Gyroscope::create(core::ILogger &logger,
                                           io::II2c &i2c,
                                           const std::uint8_t channel)
{
  const core::Result write_result_from_Ctrl1
    = i2c.writeByteToRegister(kDefaultGyroscopeAddress, kCtrl1, kConfigurationSetting1);
  if (write_result_from_Ctrl1 == core::Result::kFailure) {
    logger.log(
      core::LogLevel::kFatal,
      "Failed to configure the power mode setting in first control gyroscope at channel %d",
      channel);
    return std::nullopt;
  }
  const core::Result write_result_from_Ctrl2
    = i2c.writeByteToRegister(kDefaultGyroscopeAddress, kCtrl2, kConfigurationSetting2);
  if (write_result_from_Ctrl2 == core::Result::kFailure) {
    logger.log(core::LogLevel::kFatal,
               "Failed to configure the High pass filter in second control gyroscope at channel %d",
               channel);
    return std::nullopt;
  }
  const core::Result write_result_from_Ctrl3
    = i2c.writeByteToRegister(kDefaultGyroscopeAddress, kCtrl3, kConfigurationSetting3);
  if (write_result_from_Ctrl3 == core::Result::kFailure) {
    logger.log(
      core::LogLevel::kFatal,
      "Failed to configure the Boot and Interrupts in third control gyroscope at channel %d",
      channel);
    return std::nullopt;
  }
  const core::Result write_result_from_Ctrl5
    = i2c.writeByteToRegister(kDefaultGyroscopeAddress, kCtrl5, kConfigurationSetting5);
  if (write_result_from_Ctrl5 == core::Result::kFailure) {
    logger.log(core::LogLevel::kFatal,
               "Failed to enable FIFO in fifth control gyroscope at channel %d",
               channel);
    return std::nullopt;
  }
  return Gyroscope(logger, i2c, channel);
}

std::uint8_t Gyroscope::getChannel() const
{
  return channel_;
}

}  // namespace hyped::sensors