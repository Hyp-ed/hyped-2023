#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <io/hardware_i2c.hpp>

namespace hyped::sensors {
// Values and register names from data sheet
static constexpr std::uint8_t kTemperatureDefaultAddress = 0x38;
static constexpr std::uint8_t kCtrl                      = 0x04;
static constexpr std::uint8_t kDataTemperatureHigh       = 0x07;
static constexpr std::uint8_t kDataTemperatureLow        = 0x06;
static constexpr std::uint8_t kStatus                    = 0x05;
// The values to check the status of the temperature sensor from the 0x05 register
static constexpr std::uint8_t kBusy                       = 0x01;
static constexpr std::uint8_t kTemperatureOverUpperLimit  = 0x02;
static constexpr std::uint8_t kTemperatureUnderLowerLimit = 0x04;
// Sets the sensor to continuous mode, sets IF_ADD_INC, and sets sampling rate to 200Hz
static constexpr std::uint8_t kConfigurationSetting  = 0x3c;
static constexpr core::Float kTemperatureScaleFactor = 0.01;

class Temperature : public II2cMuxSensor<std::int16_t> {
 public:
  Temperature(core::ILogger &logger, io::HardwareI2c &i2c, const std::uint8_t channel);
  ~Temperature();

  core::Result configure();
  std::optional<std::int16_t> read();
  std::uint8_t getChannel();

 private:
  core::ILogger &logger_;
  io::HardwareI2c &i2c_;
  const std::uint8_t channel_;
};

}  // namespace hyped::sensors