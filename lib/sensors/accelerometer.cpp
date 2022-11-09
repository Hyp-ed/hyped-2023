#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(std::uint8_t mux_address, io::I2c &i2c, core::ILogger &log)
    : ADDR(mux_address),
      log_(log),
      i2c_(i2c)
{
}

Accelerometer::~Accelerometer()
{
}

std::optional<std::int16_t> getRawAccelerationX()
{
  std::optional<std::uint8_t> resultLow  = i2c_.readByte(ADDR, 0x28);
  std::optional<std::uint8_t> resultHigh = i2c_.readByte(ADDR, 0x29);

  if (!resultLow.hasValue() || !resultLow.hasValue()) { return std::nullopt; }

  std::optional<std::int16_t> rawX = (resultHigh.value() << 8) | resultLow.value();

  return rawX;
}

std::optional<std::int16_t> getRawAccelerationY()
{
  std::optional<std::uint8_t> resultLow  = i2c_.readByte(ADDR, 0x2A);
  std::optional<std::uint8_t> resultHigh = i2c_.readByte(ADDR, 0x2B);

  if (!resultLow.hasValue() || !resultLow.hasValue()) { return std::nullopt; }

  std::optional<std::int16_t> rawX = (resultHigh.value() << 8) | resultLow.value();

  return rawX;
}

std::optional<std::int16_t> getRawAccelerationZ()
{
  std::optional<std::uint8_t> resultLow  = i2c_.readByte(ADDR, 0x2C);
  std::optional<std::uint8_t> resultHigh = i2c_.readByte(ADDR, 0x2D);

  if (!resultLow.hasValue() || !resultLow.hasValue()) { return std::nullopt; }

  std::optional<std::int16_t> rawX = (resultHigh.value() << 8) | resultLow.value();

  return rawX;
}

std::optional<float> Accelerometer::read()
{
  /* check to see if the values are ready to be read*/
  std::optional<std::uint8_t> result = i2c_.readByte(ADDR, 0x27);

  // todoLater: probably have to differentiate between the two tests. The reading failing completely
  // might need to retry configuring the sensor
  if (!result.hasValue() || result.value() == 0x00) { return std::nullopt; }

  std::optional<std::int16_t> resultX = getRawAccelerationX();
  if (!resultX.hasValue()) return std::nullopt;
  std::int16_t XRawAcc
    = resultX.value() >> 2; /* shifted by 2 as 14bit resolution is used in high performance mode */
  float XAcceleration = (float)(XRawAcc);
  XAcceleration       = XAcceleration / 1000; /* mg to g */
  XAcceleration = XAcceleration * 1.952;      /* Multiply with sensitivity 1.952 in high performance
                                                 mode, 14bit, and full scale +-16g */
  printf("Acceleration X-axis %f g \r\n ", XAcceleration);

  std::optional<std::int16_t> resultY = getRawAccelerationY();
  if (!resultY.hasValue()) return std::nullopt;
  std::int16_t YRawAcc = resultY.value() >> 2;
  float YAcceleration  = (float)(YRawAcc);
  YAcceleration        = YAcceleration / 1000;
  YAcceleration        = (YAcceleration * 1.952);
  printf("Acceleration Y-axis %f g \r\n ", YAcceleration);

  std::optional<std::int16_t> resultZ = getRawAccelerationZ();
  if (!resultZ.hasValue()) return std::nullopt;
  std::int16_t ZRawAcc = resultZ.value() >> 2;
  float ZAcceleration  = (float)(ZRawAcc);
  ZAcceleration        = ZAcceleration / 1000;
  ZAcceleration        = ZAcceleration * 1.952;
  printf("Acceleration Z-axis %f g \r\n ", ZAcceleration);

  // todo: return the three values and the timestamp, ask navigation about this.
}

bool Accelerometer::configure()
{
  i2c_.writeByte(ADDR, 0x20, 0x64);
  i2c_.writeByte(ADDR, 0x21, 0xC);
  i2c_.writeByte(ADDR, 0x25, 0x30);
  return true;
}

}  // namespace hyped::sensors