#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(std::uint8_t channel, io::HardwareI2c &i2c, core::ILogger &logger)
    : channel_(channel),
      logger_(logger),
      i2c_(i2c)
{
}

Accelerometer::~Accelerometer()
{
}

std::uint8_t Accelerometer::getChannel()
{
  return channel_;
}

std::optional<std::uint16_t> Accelerometer::getRawAcceleration(Axis axis)
{
  // based on the axis asked, choose the correct register address to read from
  std::uint8_t low_byte_address, high_byte_address;
  switch (axis) {
    case Axis::x:
      low_byte_address  = kXOutL;
      high_byte_address = kXOutH;
      break;
    case Axis::y:
      low_byte_address  = kYOutL;
      high_byte_address = kYOutH;
      break;
    case Axis::z:
      low_byte_address  = kZOutL;
      high_byte_address = kZOutH;
      break;
  }

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
    = static_cast<std::uint16_t>((high_byte.value() << 8) | low_byte.value());

  return raw_acceleration;
}

std::int16_t Accelerometer::getAccelerationFromRaw(std::uint16_t rawAcc)
{
  // these values come from the data sheet. Don't chance them.
  std::int16_t acceleration
    = static_cast<std::int16_t>((static_cast<int32_t>(rawAcc) * 488) / 1000);

  return acceleration;
}

// todolater: check whether the range of values the accelerometer is correct, cause now the noise is
std::optional<core::RawAccelerationData> Accelerometer::read()
{
  /* check to see if the values are ready to be read */
  auto data_ready = i2c_.readByte(kDeviceAddress, kDataReady);
  if (!data_ready) {
    logger_.log(core::LogLevel::kFatal, "acceleration data could not be read");
    return std::nullopt;
  }
  if (data_ready.value() % 2 == 0) {
    logger_.log(core::LogLevel::kInfo, "acceleration data not ready yet to be read");
    return std::nullopt;
  }

  const auto result_x = getRawAcceleration(Axis::x);
  if (!result_x) { return std::nullopt; }
  const std::uint16_t x_raw_acc     = result_x.value();
  const std::int16_t x_acceleration = getAccelerationFromRaw(x_raw_acc);

  const auto result_y = getRawAcceleration(Axis::y);
  if (!result_y) { return std::nullopt; }
  const std::uint16_t y_raw_acc     = result_y.value();
  const std::int16_t y_acceleration = getAccelerationFromRaw(y_raw_acc);

  const auto result_z = getRawAcceleration(Axis::z);
  if (!result_z) { return std::nullopt; }
  const std::uint16_t z_raw_acc     = result_z.value();
  const std::int16_t z_acceleration = getAccelerationFromRaw(z_raw_acc);

  const std::optional<core::RawAccelerationData> acceleration_3axis{
    std::in_place,
    x_acceleration,
    y_acceleration,
    z_acceleration,
    std::chrono::system_clock::now()};

  return acceleration_3axis;
}

core::Result Accelerometer::configure()
{
  // check we are communicating with the correct sensor
  const auto device_id = i2c_.readByte(kDeviceAddress, kDevId);
  if (!device_id) {
    logger_.log(core::LogLevel::kFatal, "Failure to read device id of accelerometer");
    return core::Result::kFailure;
  }
  if (device_id.value() != expectedDevId) {
    logger_.log(core::LogLevel::kFatal, "Failure: accelerometer didn't give correct device id");
    return core::Result::kFailure;
  }

  // configure the sensor according to what was found in the repo of the manufacturers

  /* Sampling rate of 200 Hz */
  /* Enable high performance mode */
  const core::Result ctrl1_result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl1Addr, kCtrl1Value);
  if (ctrl1_result == core::Result::kFailure) return core::Result::kFailure;

  /* Enable block data update */
  /* Enable address auto increment */
  const core::Result ctrl2_result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl2Addr, kCtrl2Value);
  if (ctrl2_result == core::Result::kFailure) return core::Result::kFailure;

  /* Full scale +-16g */
  /* Filter bandwidth = ODR/2 */
  const core::Result ctrl3_result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl6Addr, kCtrl6Value);
  if (ctrl3_result == core::Result::kFailure) return core::Result::kFailure;

  return core::Result::kSuccess;
}
}  // namespace hyped::sensors