#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log) : log_(log), i2c_(i2c)
{
}

std::optional<std::uint16_t> Temperature::read()
{
  const auto Status_check = i2c_.readByte(kMux, kStatus);

  if (Status_check) {
    if (Status_check.value() == 0) {
      const std::optional<std::uint8_t> tempurature_high = i2c_.readByte(kMux, kTempuratureHigh);
      const std::optional<std::uint8_t> tempurature_low  = i2c_.readByte(kMux, kTempuratureLow);
      // getting the high and low values from the register

      if (tempurature_high.has_value() && tempurature_low.has_value()) {
        std::uint16_t tempurature = (tempurature_high.value() << 8) | tempurature_low.value();
        tempurature               = tempurature * 0.01;

        return tempurature;
      } else {
        log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
        return std::nullopt;
      }
      // Reading from the Status register to see if the tempurature is ready to be read
      // Need to have it do the correct effect and output a log of what occurred

    } else {
      log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
      return std::nullopt;
    }
  } else {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
    return std::nullopt;
  }
}

core::Result Temperature::configure()
{
  const core::Result write_result = i2c_.writeByteToRegister(kMux, kControl, 1);
  // We need to know if a begalebone has two registers set to it
  if (write_result == hyped::core::Result::kFailure) {
    log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
    return core::Result::kFailure;
  }
  // Writing to the Control register 0x04 and setting that to 1 for TEMPERATURE_1
  return core::Result::kSuccess;
}

}  // namespace hyped::sensors