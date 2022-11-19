#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(std::uint8_t mux_address, io::I2c &i2c, core::ILogger &log)
    : address_(mux_address),
      log_(log),
      i2c_(i2c)
{
}

Accelerometer::~Accelerometer()
{
}

std::optional<std::int16_t> Accelerometer::getRawAccelerationX()
{
  auto low_byte  = i2c_.readByte(address_, 0x28);
  if (!low_byte.has_value()) {
    log_.log(core::LogLevel::kFatal, "could not read register of low value of X axis for accelerometer");
    return std::nullopt;
  }

  auto high_byte = i2c_.readByte(address_, 0x29);
  if (!high_byte.has_value()) {
    log_.log(core::LogLevel::kFatal, "could not read register of high value of X axis for accelerometer");
    return std::nullopt;
  }

  auto raw_acceleration_X = (high_byte.value() << 8) | low_byte.value();

  return raw_acceleration_X;
}

std::optional<std::int16_t> Accelerometer::getRawAccelerationY()
{
  auto low_byte  = i2c_.readByte(address_, 0x2A);
  if (!low_byte.has_value()) {
    log_.log(core::LogLevel::kFatal, "could not read register of low value of Y axis for accelerometer");
    return std::nullopt;
  }

  auto high_byte = i2c_.readByte(address_, 0x2B);
  if (!high_byte.has_value()) {
    log_.log(core::LogLevel::kFatal, "could not read register of high value of Y axis for accelerometer");
    return std::nullopt;
  }

  auto raw_acceleration_Y = (high_byte.value() << 8) | low_byte.value();

  return raw_acceleration_Y;
}

std::optional<std::int16_t> Accelerometer::getRawAccelerationZ()
{
  auto low_byte  = i2c_.readByte(address_, 0x2C);
  if (!low_byte.has_value()) {
    log_.log(core::LogLevel::kFatal, "could not read register of low value of Z axis for accelerometer");
    return std::nullopt;
  }

  auto high_byte = i2c_.readByte(address_, 0x2D);
  if (!high_byte.has_value()) {
    log_.log(core::LogLevel::kFatal, "could not read register of high value of Z axis for accelerometer");
    return std::nullopt;
  }

  auto raw_acceleration_Z = (high_byte.value() << 8) | low_byte.value();

  return raw_acceleration_Z;
}

std::optional<core::acceleration_struct> Accelerometer::read()
{
  /* check to see if the values are ready to be read */
  auto dataReady = i2c_.readByte(address_, 0x27);

  if (!dataReady.has_value()) {
    log_.log(core::LogLevel::kFatal, "acceleration data could not be read");
    return std::nullopt;
  }

  // check if sensor says that data is ready to be read
  if (dataReady.value() == 0x00) {
    log_.log(core::LogLevel::kFatal, "acceleration data not ready yet to be read");
    return std::nullopt;
  }

  // x axis
  auto resultX = getRawAccelerationX();
  if (!resultX.has_value()) return std::nullopt;  
  std::int16_t XRawAcc
    = resultX.value() >> 2; /* shifted by 2 as 14bit resolution is used in high performance mode */
  core::Float XAcceleration = static_cast<core::Float>(XRawAcc);
  XAcceleration       = XAcceleration / 1000; /* mg to g */
  XAcceleration = XAcceleration * 1.952;      /* Multiply with sensitivity 1.952 in high performance
                                                 mode, 14bit, and full scale +-16g */

  // y axis
  auto resultY = getRawAccelerationY();
  if (!resultY.has_value()) return std::nullopt;

  std::int16_t YRawAcc = resultY.value() >> 2;
  core::Float YAcceleration  = static_cast<core::Float>(YRawAcc);
  YAcceleration        = YAcceleration / 1000;
  YAcceleration        = (YAcceleration * 1.952);

  // z axis
  auto resultZ = getRawAccelerationZ();
  if (!resultZ.has_value()) return std::nullopt;

  std::int16_t ZRawAcc = resultZ.value() >> 2;
  core::Float ZAcceleration  = static_cast<core::Float>(ZRawAcc);
  ZAcceleration        = ZAcceleration / 1000;
  ZAcceleration        = ZAcceleration * 1.952;

  std::optional<core::acceleration_struct> rawAcceleration{std::in_place,
                                                     XAcceleration,
                                                     YAcceleration,
                                                     ZAcceleration,
                                                     std::chrono::high_resolution_clock::now()};

  return rawAcceleration;
}

core::Result Accelerometer::configure()
{

  // if device id is incorrect, then the address is wrong
  auto deviceID = i2c_.readByte(address_, DEV_ID_REG);
  if (!deviceID.has_value()) return core::Result::kFailure;

  if (deviceID.value() != EXPECTED_DEVICE_ID) {
    log_.log(core::LogLevel::kFatal, "accelerometer didn't give correct device id");
    return core::Result::kFailure;
  } 

  /* Sampling rate of 200 Hz */
  /* Enable high performance mode */
  core::Result result1 = i2c_.writeByteToRegister(address_, CTRL_1, 0x64);
  if (result1 == core::Result::kFailure) return core::Result::kFailure;

  /* Enable block data update */
  /* Enable address auto increment */
  core::Result result2 = i2c_.writeByteToRegister(address_, CTRL_2, 0xC);
  if (result2 == core::Result::kFailure) return core::Result::kFailure;
  
  /* Full scale +-16g */
  /* Filter bandwidth = ODR/2 */
  core::Result result3 = i2c_.writeByteToRegister(address_, CTRL_6, 0x30);
  if (result3 == core::Result::kFailure) return core::Result::kFailure;

  return core::Result::kSuccess;
}

}  // namespace hyped::sensors