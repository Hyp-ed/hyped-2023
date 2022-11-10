#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kMUX    = 0x70;
static constexpr std::uint8_t kCTRL   = 0x04;
static constexpr std::uint8_t kTEMP_H = 0x07;
static constexpr std::uint8_t kTEMP_L = 0x06;
static constexpr std::uint8_t kSTATUS = 0x05;

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