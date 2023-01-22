#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(const std::uint8_t mux_address,
                             const std::uint8_t channel,
                             io::HardwareI2c &i2c,
                             core::ILogger &log)
    : mux_address_(mux_address),
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
  const auto low_byte = i2c_.readByte(mux_address_, kXOutL);
  if (!low_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the low byte for acceleration along the x-axis");
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(mux_address_, kXOutH);
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
  const auto low_byte = i2c_.readByte(mux_address_, kYOutL);
  if (!low_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the low byte for acceleration along the y-axis");
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(mux_address_, kYOutH);
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
  const auto low_byte = i2c_.readByte(mux_address_, kZOutL);
  if (!low_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the low byte for acceleration along the z-axis");
    return std::nullopt;
  }

  const auto high_byte = i2c_.readByte(mux_address_, kZOutH);
  if (!high_byte) {
    log_.log(core::LogLevel::kFatal,
             "Failed to read the high byte for acceleration along the z-axis");
    return std::nullopt;
  }

  const auto raw_acceleration_Z = (int16_t) ((high_byte.value() << 8) | low_byte.value());

  return raw_acceleration_Z;
}

// TODOLater: !!!!! this code couldn't be tested as the sensor wasn't working !!!!!!!!
std::optional<core::RawAccelerationData> Accelerometer::read()
{
  /* check to see if the values are ready to be read */
  auto dataReady = i2c_.readByte(mux_address_, kDataReady);
  if (!dataReady) {
    log_.log(core::LogLevel::kFatal, "acceleration data could not be read");
    return std::nullopt;
  }
  if (dataReady.value() % 2 == 0) {
    log_.log(core::LogLevel::kFatal, "acceleration data not ready yet to be read");
    return std::nullopt;
  }

  // x axis
  auto resultX = getRawAccelerationX();
  if (!resultX) return std::nullopt;
  const std::int16_t XRawAcc = resultX.value(); 
  core::Float XAcceleration = (static_cast<core::Float>(XRawAcc)) * 0.488f;

  // y axis
  auto resultY = getRawAccelerationY();
  if (!resultY) return std::nullopt;
  const std::int16_t YRawAcc = resultY.value(); 
  core::Float YAcceleration = (static_cast<core::Float>(YRawAcc)) * 0.488f;

  // z axis
  auto resultZ = getRawAccelerationZ();
  if (!resultZ) return std::nullopt;
  const std::int16_t ZRawAcc = resultZ.value(); 
  core::Float ZAcceleration = (static_cast<core::Float>(ZRawAcc)) * 0.488f;

  const std::optional<core::RawAccelerationData> rawAcceleration{
    std::in_place, XAcceleration, YAcceleration, ZAcceleration, std::chrono::system_clock::now()};

  return rawAcceleration;
}

core::Result Accelerometer::configure()
{
  // if device id is incorrect, then the address received is wrong
  const auto deviceID = i2c_.readByte(mux_address_, kDevId);
  if (!deviceID) {
    log_.log(core::LogLevel::kFatal, "Failure to read device id of accelerometer");
    return core::Result::kFailure;
  }
  if (deviceID.value() != expectedDevId) {
    log_.log(core::LogLevel::kFatal, "Failiure: Accelerometer didn't give correct device id");
    return core::Result::kFailure;
  }

  // configure the sensor according to the following settings :

  /* Sampling rate of 200 Hz */
  /* Enable high performance mode */
  const core::Result ctrl1Result = i2c_.writeByteToRegister(mux_address_, kCTRL1, 0x64);
  if (ctrl1Result == core::Result::kFailure) return core::Result::kFailure;

  /* Enable block data update */
  /* Enable address auto increment */
  const core::Result ctrl2Result = i2c_.writeByteToRegister(mux_address_, kCTRL2, 0xC);
  if (ctrl2Result == core::Result::kFailure) return core::Result::kFailure;

  /* Full scale +-16g */
  /* Filter bandwidth = ODR/2 */
  const core::Result ctrl3Result = i2c_.writeByteToRegister(mux_address_, kCTRL6, 0x30);
  if (ctrl3Result == core::Result::kFailure) return core::Result::kFailure;

  return core::Result::kSuccess;
}

}  // namespace hyped::sensors