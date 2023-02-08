#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

Accelerometer::~Accelerometer()
{
}

std::uint8_t Accelerometer::getChannel() const
{
  return channel_;
}

std::optional<std::int16_t> Accelerometer::getRawAcceleration(const core::Axis axis)
{
  setRegisterAddressFromAxis(axis);
  const auto low_byte = i2c_.readByte(kDefaultAccelerometerAddress, low_byte_address_);
  if (!low_byte) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to read the low byte for acceleration along the %s",
                kAxisLabels[static_cast<std::size_t>(axis)]);
    return std::nullopt;
  }
  const auto high_byte = i2c_.readByte(kDefaultAccelerometerAddress, high_byte_address_);
  if (!high_byte) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to read the high byte for acceleration along the %s",
                kAxisLabels[static_cast<std::size_t>(axis)]);
    return std::nullopt;
  }
  return static_cast<std::int16_t>((*high_byte << 8) | *low_byte);
}

void Accelerometer::setRegisterAddressFromAxis(const core::Axis axis)
{
  switch (axis) {
    case core::Axis::kX:
      low_byte_address_  = kXOutLow;
      high_byte_address_ = kXOutHigh;
      break;
    case core::Axis::kY:
      low_byte_address_  = kYOutLow;
      high_byte_address_ = kYOutHigh;
      break;
    case core::Axis::kZ:
      low_byte_address_  = kZOutLow;
      high_byte_address_ = kZOutHigh;
      break;
  }
}

std::int32_t Accelerometer::getAccelerationFromRawValue(const std::int16_t rawAcceleration)
{
  // these values come from the data sheet. Don't change them.
  return (static_cast<std::int32_t>(rawAcceleration) * 488) / 1000;
}

// TODOlater: current settings of the accelerometer make it read in +-16g but with high noise. Check
// whether other configuration could be better
std::optional<core::RawAccelerationData> Accelerometer::read()
{
  // check to see if the values are ready to be read
  const auto data_ready = i2c_.readByte(kDefaultAccelerometerAddress, kDataReady);
  if (!data_ready) {
    logger_.log(core::LogLevel::kFatal, "Failed to read acceleration data");
    return std::nullopt;
  }
  // TODOlater: here the error is not that bad, hence the return value should indicate that
  if (*data_ready % 2 == 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to read acceleration data as it is not ready");
    return std::nullopt;
  }
  const auto result_x = getRawAcceleration(core::Axis::kX);
  if (!result_x) { return std::nullopt; }
  const std::int32_t x_acceleration = getAccelerationFromRawValue(*result_x);
  const auto result_y               = getRawAcceleration(core::Axis::kY);
  if (!result_y) { return std::nullopt; }
  const std::int32_t y_acceleration = getAccelerationFromRawValue(*result_y);
  const auto result_z               = getRawAcceleration(core::Axis::kZ);
  if (!result_z) { return std::nullopt; }
  const std::int32_t z_acceleration = getAccelerationFromRawValue(*result_z);
  const std::optional<core::RawAccelerationData> acceleration_3_axis{
    std::in_place,
    x_acceleration,
    y_acceleration,
    z_acceleration,
    std::chrono::system_clock::now()};
  logger_.log(core::LogLevel::kDebug, "Successfully read accelerometer data");
  return acceleration_3_axis;
}

core::Result Accelerometer::configure()
{
  // check we are communicating with the correct sensor
  const auto device_id = i2c_.readByte(kDefaultAccelerometerAddress, kDeviceIdAddress);
  if (!device_id) {
    logger_.log(core::LogLevel::kFatal, "Failure to read device id of accelerometer");
    return core::Result::kFailure;
  }
  if (*device_id != kExpectedDeviceIdValue) {
    logger_.log(core::LogLevel::kFatal, "Failure accelerometer didn't give correct device id");
    return core::Result::kFailure;
  }
  const core::Result ctrl1_result
    = i2c_.writeByteToRegister(kDefaultAccelerometerAddress, kCtrl1Address, kCtrl1Value);
  if (ctrl1_result == core::Result::kFailure) { return core::Result::kFailure; };
  const core::Result ctrl2_result
    = i2c_.writeByteToRegister(kDefaultAccelerometerAddress, kCtrl2Address, kCtrl2Value);
  if (ctrl2_result == core::Result::kFailure) { return core::Result::kFailure; };
  const core::Result ctrl6_result
    = i2c_.writeByteToRegister(kDefaultAccelerometerAddress, kCtrl6Address, kCtrl6Value);
  if (ctrl6_result == core::Result::kFailure) { return core::Result::kFailure; };
  return core::Result::kSuccess;
}

}  // namespace hyped::sensors