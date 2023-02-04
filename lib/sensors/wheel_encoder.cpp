#include "WheelEncoder.hpp"

namespace hyped::sensors {

// TODOLater: here finish the implementation once other i2c instance has been made
std::optional<WheelEncoder> WheelEncoder::create(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
{
}

WheelEncoder::WheelEncoder(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

WheelEncoder::~WheelEncoder()
{
}

std::optional<std::uint24_t> WheelEncoder::getWheelTurnCount()
{
  const auto low_byte = i2c_.readByte(kDefaultWheelEncoderAddress, low_byte_address_);
  if (!low_byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read the low byte for wheel encoder")
    return std::nullopt;
  }

  const auto middle_byte = i2c_.readByte(kDefaultWheelEncoderAddress, middle_byte_address_);
  if (!middle_byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read the middle byte for wheel encoder")
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(kDefaultWheelEncoderAddress, middle_byte_address_);
  if (!high_byte) {
    logger_.log(core::LogLevel::kFatal, "Failed to read the high byte for wheel encoder")
    return std::nullopt;
  }

  return static_cast<std::int24_t>((*high_byte << 16) | (*middle_byte << 8) | *low_byte);
}

core::Result WheelEncoder::resetWheelTurnCount()
{
  const core::Result reset_result = i2c_.writeByteToRegister(kDefaultWheelEncoderAddress, kFreeRegisterAddress, kFreeRegisterResetValue);
  if (reset_result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal, "Failed to reset count on the wheel encoder")
    return core::Result::kFailure;
  };
  return core::Result.kSuccess;
}

}  // namespace hyped::sensors