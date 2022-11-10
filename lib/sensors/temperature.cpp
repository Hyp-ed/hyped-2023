#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log) : log_(log), i2c_(i2c)
{
}

std::optional<std::uint16_t> Temperature::read()
{
  if (i2c_.readByte(TEMPERATURE_1, STATUS) == 0) {
    const std::optional<std::uint8_t> Tempurature_High = i2c_.readByte(kMUX, TEMP_H);
    const std::optional<std::uint8_t> Tempurature_Low  = i2c_.readByte(kMUX, TEMP_L);
    // getting the high and low values from the register

    if (Tempurature_High = !std::nullopt, Tempurature_Low = !std::nullopt) {
      std::optional<std::uint16_t> Tempurature = (Tempurature_High << 8) | Tempurature_Low;
      Tempurature                              = Tempurature * 0.01;

      return Tempurature;
    } else {
      log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
    }
    // Reading from the Status register to see if the tempurature is ready to be read
    // Need to have it do the correct effect and output a log of what occurred
  }
  // log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
}

bool Temperature::configure()
{
  hyped::core::Result::Check = i2c_.writeByte(kMUX, CTRL, 1);
  // We need to know if a begalebone has two registers set to it
  if (Check == hyped::core::Result::kFailure) { return false; }
  // Writing to the Control register 0x04 and setting that to 1 for TEMPERATURE_1

  // log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
  return true;
}

}  // namespace hyped::sensors