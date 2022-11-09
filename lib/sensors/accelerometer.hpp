#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {


struct g_result {
  float x;
  float y;
  float z;
  std::chrono::time_point<std::chrono::system_clock> time;
};

class Accelerometer : II2cSensor<float> {
 public:
  Accelerometer(const std::uint8_t mux_address, io::I2c &i2c, hyped::core::ILogger &log);
  ~Accelerometer();

  bool configure();
  std::optional<g_result> read();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;  // I2c object for the accelerometer
  const std::unit8_t ADDR;

  std::optional<std::int16_t> getRawAccelerationX();
  std::optional<std::int16_t> getRawAccelerationY();
  std::optional<std::int16_t> getRawAccelerationZ();
};

}  // namespace hyped::sensors