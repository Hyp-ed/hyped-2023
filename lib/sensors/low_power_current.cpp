#include <low_power_current.hpp>

namespace hyped::sensors {

LowPowerCurrent::LowPowerCurrent(core::ILogger &logger,
                                 std::shared_ptr<io::II2c> i2c,
                                 const std::uint8_t device_address)
    : logger_(logger),
      i2c_(i2c),
      device_address_(device_address)
{
}

LowPowerCurrent::~LowPowerCurrent()
{
}

std::optional<core::Float> LowPowerCurrent::readCurrent()
{
  const auto byte = i2c_->readByte(device_address_, kLowPowerCurrectRegister);
  if (!byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read current on channel %d", device_address_);
    return std::nullopt;
  }
  logger_.log(core::LogLevel::kDebug, "Current read on channel %d successful", device_address_);
  // Divide milliAmps output by 1,000 to return Amps value
  return static_cast<core::Float>(*byte) / 1000;
}

std::uint8_t LowPowerCurrent::getDeviceAddress() const
{
  return device_address_;
}

}  // namespace hyped::sensors