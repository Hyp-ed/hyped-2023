#include <low_power_current.hpp>

namespace hyped::sensors {

std::optional<LowPowerCurrent> LowPowerCurrent::create(core::ILogger &logger,
                                                       io::II2c &i2c,
                                                       std::uint8_t channel = kdefault_i2c_address)
{
  return LowPowerCurrent(logger, i2c, channel);
}

LowPowerCurrent::LowPowerCurrent(core::ILogger &logger, io::II2c &i2c, std::uint8_t channel)
    : logger_(logger),
      channel_(channel),
      i2c_(i2c){};

LowPowerCurrent::~LowPowerCurrent(){};

std::optional<float> LowPowerCurrent::readCurrent()
{
  if (!(i2c_.readByte(channel_, kcurrent_reg))) {
    logger_.log(core::LogLevel::kFatal, "Failed to read current on channel %d", channel_);
    return std::nullopt;
  }
  logger_.log(core::LogLevel::kDebug, "Current read on channel %d successful", channel_);
  return i2c_.readByte(channel_, kcurrent_reg);
};

}  // namespace hyped::sensors