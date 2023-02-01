#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kGyroscope             = 0x69;
static constexpr std::uint8_t kDataXH                = 0x29;
static constexpr std::uint8_t kDataXL                = 0x28;
static constexpr std::uint8_t kDataYH                = 0x2B;
static constexpr std::uint8_t kDataYL                = 0x2A;
static constexpr std::uint8_t kDataZH                = 0x2D;
static constexpr std::uint8_t kDataZL                = 0x2C;
static constexpr std::uint8_t kCtrl1                 = 0x20;
static constexpr std::uint8_t kCtrl2                 = 0x21;
static constexpr std::uint8_t kCtrl3                 = 0x22;
static constexpr std::uint8_t kCtrl5                 = 0x24;
static constexpr std::uint8_t kConfigurationSetting1 = 0xff;
static constexpr std::uint8_t kConfigurationSetting2 = 0x20;
static constexpr std::uint8_t kConfigurationSetting3 = 0xff;
static constexpr std::uint8_t kConfigurationSetting5 = 0x40;

class Gyroscope : public II2cMuxSensor<std::int16_t> {
 public:
  Gyroscope(hyped::core::ILogger &log, io::I2c &i2c, const std::uint8_t channel);
  ~Gyroscope();

  core::Result configure();
  std::optional<std::int16_t> read();
  std::uint8_t getChannel();

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c &i2c_;
  const std::uint8_t channel_;
};

}  // namespace hyped::sensors