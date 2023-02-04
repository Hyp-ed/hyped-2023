#pragma once

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

// The datasheet lists four possible I2C addresses; 0x40 has been defined as the default
static constexpr std::uint8_t kdefault_i2c_address = 0x40;
static constexpr std::uint8_t ki2c_address_2       = 0x41;
static constexpr std::uint8_t ki2c_address_3       = 0x44;
static constexpr std::uint8_t ki2c_address_4       = 0x45;
// Registers with reference to the INA219 driver code
static constexpr std::uint8_t kcurrent_reg = 0x04;

class LowPowerCurrent {
 public:
  static std::optional<LowPowerCurrent> create(core::ILogger &logger,
                                               io::II2c &i2c,
                                               std::uint8_t channel);

  ~LowPowerCurrent();

  std::optional<float> readCurrent();

 private:
  LowPowerCurrent(core::ILogger &logger, io::II2c &i2c, std::uint8_t channel);

 private:
  core::ILogger &logger_;
  io::II2c &i2c_;
  std::uint8_t channel_;
};
}  // namespace hyped::sensors