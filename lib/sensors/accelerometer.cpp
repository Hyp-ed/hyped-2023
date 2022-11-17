#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(std::uint8_t mux_address_ess, io::I2c &i2c, core::ILogger &log)
    : address_ess_(mux_address_ess),
      log_(log),
      i2c_(i2c)
{
}

Accelerometer::~Accelerometer()
{
}

std::optional<std::int16_t> getRawAccelerationX()
{
  std::optional<std::uint8_t> resultLow  = i2c_.readByte(address_, 0x28);
  std::optional<std::uint8_t> resultHigh = i2c_.readByte(address_, 0x29);

  if (!resultLow.hasValue() || !resultLow.hasValue()) { return std::nullopt; }

  std::optional<std::int16_t> rawX = (resultHigh.value() << 8) | resultLow.value();

  return rawX;
}

std::optional<std::int16_t> getRawAccelerationY()
{
  std::optional<std::uint8_t> resultLow  = i2c_.readByte(address_, 0x2A);
  std::optional<std::uint8_t> resultHigh = i2c_.readByte(address_, 0x2B);

  if (!resultLow.hasValue() || !resultLow.hasValue()) { return std::nullopt; }

  std::optional<std::int16_t> rawX = (resultHigh.value() << 8) | resultLow.value();

  return rawX;
}

std::optional<std::int16_t> getRawAccelerationZ()
{
  std::optional<std::uint8_t> resultLow  = i2c_.readByte(address_, 0x2C);
  std::optional<std::uint8_t> resultHigh = i2c_.readByte(address_, 0x2D);

  if (!resultLow.hasValue() || !resultLow.hasValue()) { return std::nullopt; }

  std::optional<std::int16_t> rawX = (resultHigh.value() << 8) | resultLow.value();

  return rawX;
}

std::optional<acceleration_struct> Accelerometer::read()
{
  /* check to see if the values are ready to be read */
  std::optional<std::uint8_t> result = i2c_.readByte(address_, 0x27);

  if (!result.hasValue()) {
    return std::nullopt;
  }

  // check if sensor says that data is ready to be read
  if (result.value() == 0x00) {
    log_.log(hyped::core::LogLevel::kLow, "acceleration data not ready yet to be read");
    return std::nullopt;
  }

  std::optional<acceleration_struct> result;


  // x axis
  std::optional<std::int16_t> resultX = getRawAccelerationX();
  if (!resultX.hasValue()) return std::nullopt;

  std::int16_t XRawAcc = resultX.value() >> 2; /* shifted by 2 as 14bit resolution is used in high performance mode */
  float XAcceleration = (float)(XRawAcc);
  XAcceleration       = XAcceleration / 1000; /* mg to g */
  XAcceleration = XAcceleration * 1.952;      /* Multiply with sensitivity 1.952 in high performance
                                                 mode, 14bit, and full scale +-16g */
  result.x = XAcceleration;
  
  // y axis
  std::optional<std::int16_t> resultY = getRawAccelerationY();
  if (!resultY.hasValue()) return std::nullopt;

  std::int16_t YRawAcc = resultY.value() >> 2;
  float YAcceleration  = (float)(YRawAcc);
  YAcceleration        = YAcceleration / 1000;
  YAcceleration        = (YAcceleration * 1.952);
  result.y = YAcceleration;

  // z axis
  std::optional<std::int16_t> resultZ = getRawAccelerationZ();
  if (!resultZ.hasValue()) return std::nullopt;

  std::int16_t ZRawAcc = resultZ.value() >> 2;
  float ZAcceleration  = (float)(ZRawAcc);
  ZAcceleration        = ZAcceleration / 1000;
  ZAcceleration        = ZAcceleration * 1.952;
  result.z = ZAcceleration;

  // associate the reading with the time they were made
  result.time = std::chrono::high_resolution_clock::now();
  
  return retult;
}

bool Accelerometer::configure()
{
  i2c_.writeByte(address_, CTRL_1, 0x64);
  i2c_.writeByte(address_, CTRL_2, 0xC);
  i2c_.writeByte(address_, CTRL_6, 0x30);

  // nothing can go wrong
  return true;
}

}  // namespace hyped::sensors