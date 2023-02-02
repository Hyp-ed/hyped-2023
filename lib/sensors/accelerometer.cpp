#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(core::ILogger &logger,
                             io::HardwareI2c &i2c,
                             const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

Accelerometer::~Accelerometer()
{
}

std::uint8_t Accelerometer::getChannel()
{
  return channel_;
}

std::optional<std::int16_t> Accelerometer::getRawAcceleration(const Axis axis)
{
  std::uint8_t low_byte_address, high_byte_address;
  setRegisterAddressFromAxis(axis, &low_byte_address, &high_byte_address);
  const auto low_byte = i2c_.readByte(kDeviceAddress, low_byte_address);
  if (!low_byte) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to read the low byte for acceleration along the %s",
                AxisStrings[axis]);
    return std::nullopt;
  }
  const auto high_byte = i2c_.readByte(kDeviceAddress, high_byte_address);
  if (!high_byte) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to read the low byte for acceleration along the %s",
                AxisStrings[axis]);
    return std::nullopt;
  }
  const auto raw_acceleration
    = static_cast<std::int16_t>((high_byte.value() << 8) | low_byte.value());
  return raw_acceleration;
}

void Accelerometer::setRegisterAddressFromAxis(const Axis axis,
                                               std::uint8_t *low_byte_address,
                                               std::uint8_t *high_byte_address)
{
  switch (axis) {
    case Axis::x:
      *low_byte_address  = kXOutLow;
      *high_byte_address = kXOutHigh;
      break;
    case Axis::y:
      *low_byte_address  = kYOutLow;
      *high_byte_address = kYOutHigh;
      break;
    case Axis::z:
      *low_byte_address  = kZOutLow;
      *high_byte_address = kZOutHigh;
      break;
  }
}

std::int16_t Accelerometer::getAccelerationFromRaw(const std::int16_t rawAcceleration)
{
  // these values come from the data sheet. Don't change them.
  std::int16_t acceleration
    = static_cast<std::int16_t>((static_cast<int32_t>(rawAcceleration) * 488) / 1000);
  return acceleration;
}

// TODOlater: current settings of the accelerometer make it read in +-16g but with high noise. Check
// whether other configuration could be better
std::optional<core::RawAccelerationData> Accelerometer::read()
{
  // check to see if the values are ready to be read
  const auto data_ready = i2c_.readByte(kDeviceAddress, kDataReady);
  if (!data_ready) {
    logger_.log(core::LogLevel::kFatal, "Failed to read acceleration data");
    return std::nullopt;
  }
  // TODOlater: here the error is not that bad, hence the return value should indicate that
  if (data_ready.value() % 2 == 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to read acceleration data as it is not ready");
    return std::nullopt;
  }
  const auto result_x = getRawAcceleration(Axis::x);
  if (!result_x) { return std::nullopt; }
  const std::int16_t x_raw_acceleration = result_x.value();
  const std::int16_t x_acceleration     = getAccelerationFromRaw(x_raw_acceleration);
  const auto result_y                   = getRawAcceleration(Axis::y);
  if (!result_y) { return std::nullopt; }
  const std::int16_t y_raw_acceleration = result_y.value();
  const std::int16_t y_acceleration     = getAccelerationFromRaw(y_raw_acceleration);
  const auto result_z                   = getRawAcceleration(Axis::z);
  if (!result_z) { return std::nullopt; }
  const std::int16_t z_raw_acceleration = result_z.value();
  const std::int16_t z_acceleration     = getAccelerationFromRaw(z_raw_acceleration);
  const std::optional<core::RawAccelerationData> acceleration_3axis{
    std::in_place,
    x_acceleration,
    y_acceleration,
    z_acceleration,
    std::chrono::system_clock::now()};
  logger_.log(core::LogLevel::kDebug, "Successfully read accelerometer data");
  return acceleration_3axis;
}

core::Result Accelerometer::configure()
{
  // check we are communicating with the correct sensor
  const auto device_id = i2c_.readByte(kDeviceAddress, kDeviceId);
  if (!device_id) {
    logger_.log(core::LogLevel::kFatal, "Failure to read device id of accelerometer");
    return core::Result::kFailure;
  }
  if (device_id.value() != kExpectedDeviceId) {
    logger_.log(core::LogLevel::kFatal, "Failure: accelerometer didn't give correct device id");
    return core::Result::kFailure;
  }
  // Sampling rate of 200 Hz
  // Enable high performance mode
  const core::Result ctrl1_result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl1Address, kCtrl1Value);
  if (ctrl1_result == core::Result::kFailure) { return core::Result::kFailure; };
  // Enable block data update
  // Enable address auto increment
  const core::Result ctrl2_result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl2Address, kCtrl2Value);
  if (ctrl2_result == core::Result::kFailure) { return core::Result::kFailure; };
  // Full scale +-16g
  // Filter bandwidth = ODR/2
  const core::Result ctrl6_result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl6Address, kCtrl6Value);
  if (ctrl6_result == core::Result::kFailure) { return core::Result::kFailure; };
  return core::Result::kSuccess;
}
}  // namespace hyped::sensors