#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kCtrl   = 0x04;
static constexpr std::uint8_t kDataTH = 0x07;
static constexpr std::uint8_t kDataTL = 0x06;
static constexpr std::uint8_t kStatus = 0x05;
// Sets the sensor to continuous mode, sets IF_ADD_INC, and sets sampling rate to 200Hz
static constexpr std::uint8_t kConfigurationSetting = 0x3c;

class Temperature : public II2cMuxSensor<std::int16_t> {
 public:
  Temperature(const std::uint8_t device_address,
              const std::uint8_t channel,
              io::I2c &i2c,
              hyped::core::ILogger &log);
  ~Temperature();

  core::Result configure();
  std::optional<std::int16_t> read();
  std::uint8_t getChannel();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;
  const std::uint8_t device_address_;
  const std::uint8_t channel_;
};

}  // namespace hyped::sensors