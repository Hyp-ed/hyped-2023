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

std::optional<std::kCtrl2Value> Accelerometer::getRawAcceleration(Axis axis)
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
    = static_cast<std::int16_t>((high_byte.value() << 8) | low_byte.value());

  return raw_acceleration;
}

std::int16_t Accelerometer::getAccelerationFromRaw(std::int16_t rawAcc)
{
  // ! these values come from the data sheet. Don't chance them.
  std::int16_t acceleration = static_cast<std::int16_t>((((int32_t)rawAcc) * 488) / 1000);

  return acceleration;
}

// todolater: check whether the range of values the accelerometer is correct, cause now the noise is
std::optional<core::RawAccelerationData> Accelerometer::read()
{
  /* check to see if the values are ready to be read */
  auto dataReady = i2c_.readByte(kDeviceAddress, kDataReady);
  if (!dataReady) {
    logger_.log(core::LogLevel::kFatal, "acceleration data could not be read");
    return std::nullopt;
  }
  if (dataReady.value() % 2 == 0) {
    logger_.log(core::LogLevel::kInfo, "acceleration data not ready yet to be read");
    return std::nullopt;
  }

  // x axis
  const auto resultX = getRawAcceleration(Axis::x);
  if (!resultX) return std::nullopt;
  const std::int16_t XRawAcc       = resultX.value();
  const std::int16_t XAcceleration = getAccelerationFromRaw(XRawAcc);

  // y axis
  const auto resultY = getRawAcceleration(Axis::y);
  if (!resultY) return std::nullopt;
  const std::int16_t YRawAcc       = resultY.value();
  const std::int16_t YAcceleration = getAccelerationFromRaw(YRawAcc);

  // z axis
  const auto resultZ = getRawAcceleration(Axis::z);
  if (!resultZ) return std::nullopt;
  const std::int16_t ZRawAcc       = resultZ.value();
  const std::int16_t ZAcceleration = getAccelerationFromRaw(ZRawAcc);

  const std::optional<core::RawAccelerationData> Acceleration3D{
    std::in_place, XAcceleration, YAcceleration, ZAcceleration, std::chrono::system_clock::now()};

  return Acceleration3D;
}

core::Result Accelerometer::configure()
{
  // check we are communicating with the correct sensor
  const auto deviceID = i2c_.readByte(kDeviceAddress, kDevId);
  if (!deviceID) {
    logger_.log(core::LogLevel::kFatal, "Failure to read device id of accelerometer");
    return core::Result::kFailure;
  }
  if (deviceID.value() != expectedDevId) {
    logger_.log(core::LogLevel::kFatal, "Failure: accelerometer didn't give correct device id");
    return core::Result::kFailure;
  }

  // configure the sensor according to what was found in the repo of the manufacturers

  /* Sampling rate of 200 Hz */
  /* Enable high performance mode */
  const core::Result ctrl1Result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl1Addr, kCtrl1Value);
  if (ctrl1Result == core::Result::kFailure) return core::Result::kFailure;

  /* Enable block data update */
  /* Enable address auto increment */
  const core::Result ctrl2Result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl2Addr, kCtrl2Value);
  if (ctrl2Result == core::Result::kFailure) return core::Result::kFailure;

  /* Full scale +-16g */
  /* Filter bandwidth = ODR/2 */
  const core::Result ctrl3Result
    = i2c_.writeByteToRegister(kDeviceAddress, kCtrl6Addr, kCtrl6Value);
  if (ctrl3Result == core::Result::kFailure) return core::Result::kFailure;

  return core::Result::kSuccess;
}
}  // namespace hyped::sensors