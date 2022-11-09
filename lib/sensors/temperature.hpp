#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

const std::uint8_t CTRL          = 0x04;
const std::uint8_t TEMPERATURE_1 = 0x38;
const std::uint8_t TEMPERATURE_2 = 0x3f;
const std::uint8_t TEMP_H        = 0x07;
const std::uint8_t TEMP_L        = 0x06;
const std::uint8_t STATUS        = 0x05;

class Temperature : public II2cSensor<std::uint16_t> {
 public:
  Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log);
  ~Temperature();

  bool configure();
  std::optional<std::uint16_t> read();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;
};

}  // namespace hyped::sensors