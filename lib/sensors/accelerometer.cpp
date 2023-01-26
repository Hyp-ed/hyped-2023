#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(const std::uint8_t kDeviceAddr,
                             const std::uint8_t channel,
                             io::HardwareI2c &i2c,
                             core::ILogger &log)
    : kDeviceAddr(kDeviceAddr),
      channel_(channel),
      log_(log),
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

std::optional<std::int16_t> Accelerometer::getRawAccelerationX()
{
  const auto low_byte = i2c_.readByte(kDeviceAddr, kXOutL);
  if (!low_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the low byte for acceleration along the x-axis");
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(kDeviceAddr, kXOutH);
  if (!high_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the high byte for acceleration along the x-axis");
    return std::nullopt;
  }

  const auto raw_acceleration_X = (int16_t) ((high_byte.value() << 8) | low_byte.value());

  return raw_acceleration_X;
}

std::optional<std::int16_t> Accelerometer::getRawAccelerationY()
{
  const auto low_byte = i2c_.readByte(kDeviceAddr, kYOutL);
  if (!low_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the low byte for acceleration along the y-axis");
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(kDeviceAddr, kYOutH);
  if (!high_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the high byte for acceleration along the y-axis");
    return std::nullopt;
  }

  const auto raw_acceleration_Y = (int16_t) ((high_byte.value() << 8) | low_byte.value());

  return raw_acceleration_Y;
}

std::optional<std::int16_t> Accelerometer::getRawAccelerationZ()
{
  const auto low_byte = i2c_.readByte(kDeviceAddr, kZOutL);
  if (!low_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the low byte for acceleration along the z-axis");
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(kDeviceAddr, kZOutH);
  if (!high_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the high byte for acceleration along the z-axis");
    return std::nullopt;
  }

  const auto raw_acceleration_Z = (int16_t) ((high_byte.value() << 8) | low_byte.value());

  return raw_acceleration_Z;
}

// todolater: check whether the range of values the accelerometer is correct, cause now the noise is big
std::optional<core::RawAccelerationData> Accelerometer::read()
{
  /* check to see if the values are ready to be read */
  auto dataReady = i2c_.readByte(kDeviceAddr, kDataReady);
  if (!dataReady) {
    log_.log(core::LogLevel::kFatal, "acceleration data could not be read");
    return std::nullopt;
  }
  if (dataReady.value() % 2 == 0) {
    log_.log(core::LogLevel::kInfo, "acceleration data not ready yet to be read");
    return std::nullopt;
  }

  // x axis
  auto resultX = getRawAccelerationX();
  if (!resultX) return std::nullopt;
  const std::int16_t XRawAcc = resultX.value(); 
  int16_t XAcceleration = (int16_t) ((((int32_t) XRawAcc) * 488) / 1000);

  // y axis
  auto resultY = getRawAccelerationY();
  if (!resultY) return std::nullopt;
  const std::int16_t YRawAcc = resultY.value(); 
  int16_t YAcceleration = (int16_t) ((((int32_t) YRawAcc) * 488) / 1000);

  // z axis
  auto resultZ = getRawAccelerationZ();
  if (!resultZ) return std::nullopt;
  const std::int16_t ZRawAcc = resultZ.value(); 
  int16_t ZAcceleration = (int16_t) ((((int32_t) ZRawAcc) * 488) / 1000);

  const std::optional<core::RawAccelerationData> rawAcceleration{
    std::in_place, XAcceleration, YAcceleration, ZAcceleration, std::chrono::system_clock::now()};

  return rawAcceleration;
}

core::Result Accelerometer::configure()
{
  // check we are communicating with the correct sensor
  const auto deviceID = i2c_.readByte(kDeviceAddr, kDevId);
  if (!deviceID) {
    log_.log(core::LogLevel::kFatal, "Failure to read device id of accelerometer");
    return core::Result::kFailure;
  }
  if (deviceID.value() != expectedDevId) {
    log_.log(core::LogLevel::kFatal, "Failure: accelerometer didn't give correct device id");
    return core::Result::kFailure;
  }

  // configure the sensor according to what was found in the repo of the manufacturers 

  /* Sampling rate of 200 Hz */
  /* Enable high performance mode */
  const core::Result ctrl1Result = i2c_.writeByteToRegister(kDeviceAddr, kCTRL1Addr, kCTRL1Value);
  if (ctrl1Result == core::Result::kFailure) return core::Result::kFailure;

  /* Enable block data update */
  /* Enable address auto increment */
  const core::Result ctrl2Result = i2c_.writeByteToRegister(kDeviceAddr, kCTRL2Addr, kCTRL2Value);
  if (ctrl2Result == core::Result::kFailure) return core::Result::kFailure;

  /* Full scale +-16g */
  /* Filter bandwidth = ODR/2 */
  const core::Result ctrl3Result = i2c_.writeByteToRegister(kDeviceAddr, kCTRL6Addr, kCTRL6Value);
  if (ctrl3Result == core::Result::kFailure) return core::Result::kFailure;

  return core::Result::kSuccess;
}

}  // namespace hyped::sensors