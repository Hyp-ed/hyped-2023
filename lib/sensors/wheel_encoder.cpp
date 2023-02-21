#include "wheel_encoder.hpp"

namespace hyped::sensors {

std::optional<WheelEncoder> WheelEncoder::create(core::ILogger &logger,
                                                 std::shared_ptr<io::II2c> i2c,
                                                 const std::uint8_t channel)
{
  // resetWheelTurnCount();
  return WheelEncoder(logger, i2c, channel);
}

WheelEncoder::WheelEncoder(core::ILogger &logger,
                           std::shared_ptr<io::II2c> i2c,
                           const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

WheelEncoder::~WheelEncoder()
{
}

// TODOLater: do testing once we have the sensor !
std::optional<std::uint32_t> WheelEncoder::getWheelTurnCount()
{
  const auto low_byte = i2c_->readByte(kDefaultWheelEncoderAddress, low_byte_address_);
  if (!low_byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read the low byte for wheel encoder");
    return std::nullopt;
  }

  const auto middle_byte = i2c_->readByte(kDefaultWheelEncoderAddress, middle_byte_address_);
  if (!middle_byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read the middle byte for wheel encoder");
    return std::nullopt;
  }

  const auto high_byte = i2c_->readByte(kDefaultWheelEncoderAddress, middle_byte_address_);
  if (!high_byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read the high byte for wheel encoder");
    return std::nullopt;
  }

  return static_cast<std::uint32_t>((*high_byte << 16) | (*middle_byte << 8) | *low_byte);
}

core::Result WheelEncoder::resetWheelTurnCount()
{
  const core::Result reset_result = i2c_->writeByteToRegister(
    kDefaultWheelEncoderAddress, kFreeRegisterAddress, kFreeRegisterValue);
  if (reset_result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal, "Failed to reset count on the wheel encoder");
    return core::Result::kFailure;
  };
  return core::Result::kSuccess;
}

}  // namespace hyped::sensors