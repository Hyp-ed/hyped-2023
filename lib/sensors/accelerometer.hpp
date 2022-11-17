#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include "core/types.hpp"
#include <io/i2c.hpp>

namespace hyped::sensors {

class Accelerometer : II2cMuxSensor<core::acceleration_struct> {
 public:
  Accelerometer(const std::uint8_t mux_address, io::I2c &i2c, core::ILogger &log);
  ~Accelerometer();

  core::Result configure();
  std::optional<core::acceleration_struct> read();

 private:

  const std::uint8_t CTRL_1 = 0x20;
  const std::uint8_t CTRL_2 = 0x21;
  const std::uint8_t CTRL_6 = 0x25;

  core::ILogger &log_;
  io::I2c &i2c_;  // I2c object for the accelerometer
  const std::uint8_t address_;

  std::optional<std::int16_t> getRawAccelerationX();
  std::optional<std::int16_t> getRawAccelerationY();
  std::optional<std::int16_t> getRawAccelerationZ();
};

}  // namespace hyped::sensors