#pragma once

#include "i2c_sensors.hpp"

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/time.hpp>
#include <io/i2c.hpp>

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
  Temperature(core::ILogger &logger,
              core::ITimeSource &time_source,
              std::shared_ptr<io::II2c> i2c,
              const std::uint8_t channel);
  ~Temperature();

  virtual core::Result configure();
  virtual std::optional<core::Measurement<std::int16_t>> read();
  virtual std::uint8_t getChannel() const;

 private:
  core::ILogger &logger_;
  core::ITimeSource &time_source_;
  std::shared_ptr<io::II2c> i2c_;
  const std::uint8_t channel_;
};

}  // namespace hyped::sensors
