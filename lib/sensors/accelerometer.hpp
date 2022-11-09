#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors
{

  const uint8_t ADDR = 0x19;


  class Accelerometer : II2cSensor<float>
  {
  public:
    Accelerometer(io::I2c &i2c, hyped::core::ILogger &log);
    ~Accelerometer();

    bool configure();
    std::optional<float> read();

  private:
    hyped::core::ILogger &log_;
    hyped::io::I2c &i2c_; // I2c object for the accelerometer
  };

} // namespace hyped::sensors