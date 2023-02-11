#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

// Registers taken from the data sheet
constexpr std::uint8_t kDefaultGyroscopeAddress = 0x69;
constexpr std::uint8_t kDataXHigh               = 0x29;
constexpr std::uint8_t kDataXLow                = 0x28;
constexpr std::uint8_t kDataYHigh               = 0x2B;
constexpr std::uint8_t kDataYLow                = 0x2A;
constexpr std::uint8_t kDataZHigh               = 0x2D;
constexpr std::uint8_t kDataZLow                = 0x2C;
constexpr std::uint8_t kCtrl1                   = 0x20;
constexpr std::uint8_t kCtrl2                   = 0x21;
constexpr std::uint8_t kCtrl3                   = 0x22;
constexpr std::uint8_t kCtrl5                   = 0x24;
constexpr std::uint8_t kConfigurationSetting1   = 0xff;
constexpr std::uint8_t kConfigurationSetting2   = 0x20;
constexpr std::uint8_t kConfigurationSetting3   = 0xff;
constexpr std::uint8_t kConfigurationSetting5   = 0x40;

class Gyroscope {
 public:
  static std::optional<Gyroscope> create(core::ILogger &logger,
                                         std::shared_ptr<io::II2c> i2c,
                                         const std::uint8_t channel,
                                         const std::uint8_t device_address
                                         = kDefaultGyroscopeAddress);
  ~Gyroscope();

  const std::optional<std::int16_t> read(core::Axis axis);
  std::uint8_t getChannel() const;

 private:
  Gyroscope(core::ILogger &logger,
            std::shared_ptr<io::II2c> i2c,
            const std::uint8_t channel,
            const std::uint8_t device_address);

  core::ILogger &logger_;
  std::shared_ptr<io::II2c> i2c_;
  const std::uint8_t channel_;
  const std::uint8_t device_address_;
};

}  // namespace hyped::sensors