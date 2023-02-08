#include <low_power_current.hpp>

namespace hyped::sensors {

std::optional<LowPowerCurrent> LowPowerCurrent::create(core::ILogger &logger,
                                                       io::II2c &i2c,
                                                       const std::uint8_t channel
                                                       = kDefaultI2cAddress)
{
  return LowPowerCurrent(logger, i2c, channel);
}

LowPowerCurrent::LowPowerCurrent(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
    : logger_(logger),
      channel_(channel),
      i2c_(i2c){};

LowPowerCurrent::~LowPowerCurrent(){};

std::optional<core::Float> LowPowerCurrent::readCurrent()
{
  const auto byte = i2c_.readByte(channel_, kCurrentReg);
  if (!byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read current on channel %d", channel_);
    return std::nullopt;
  }
  logger_.log(core::LogLevel::kDebug, "Current read on channel %d successful", channel_);
  // Return current in Amps
  return (core::Float)byte.value() / kAmpsDivisor;
};

std::uint8_t LowPowerCurrent::getChannel() const
{
  return channel_;
};

}  // namespace hyped::sensors