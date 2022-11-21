#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kCtrl                 = 0x04;
static constexpr std::uint8_t kData_T_H             = 0x07;
static constexpr std::uint8_t kData_T_L             = 0x06;
static constexpr std::uint8_t kStatus               = 0x05;
static constexpr std::uint8_t kConfigurationSetting = 0x3c;

class Temperature : public II2cMuxSensor<std::uint16_t> {
 public:
  Temperature(const std::uint8_t mux_address, hyped::io::I2c &i2c, hyped::core::ILogger &log);
  ~Temperature();

  core::Result configure();
  std::optional<std::uint16_t> read();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;
  const std::uint8_t mux_address_;
};

}  // namespace hyped::sensors