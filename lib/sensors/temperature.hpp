#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kMux             = 0x70;
static constexpr std::uint8_t kControl         = 0x04;
static constexpr std::uint8_t kTempuratureHigh = 0x07;
static constexpr std::uint8_t kTempuratureLow  = 0x06;
static constexpr std::uint8_t kStatus          = 0x05;

class Temperature : public II2cMuxSensor<std::uint16_t> {
 public:
  Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log);
  ~Temperature();

  core::Result configure();
  std::optional<std::uint16_t> read();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;
};

}  // namespace hyped::sensors