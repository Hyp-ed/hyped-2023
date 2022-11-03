#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

class Temperature : public II2cSensor<uint16_t> {
 public:
  Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log);
  ~Temperature();

  bool configure();
  std::optional<uint16_t> read();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;
};

}  // namespace hyped::sensors