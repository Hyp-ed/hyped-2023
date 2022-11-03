#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

class Accelerometer : II2cSensor<float> {
  public:
    Accelerometer(io::I2c &i2c, hyped::core::ILogger &log);
    ~Accelerometer();

    bool configure(); 
    std::optional<float> read();

  private:
    hyped::core::ILogger &log_; 
    hyped::io::I2c &i2c_; 

};

}  // namespace hyped::sensors