#pragma once

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

// The datasheet lists four possible I2C addresses; 0x40 has been defined as the default
constexpr std::uint8_t kDefaultI2cAddress = 0x40;
constexpr std::uint8_t kI2cAddress2       = 0x41;
constexpr std::uint8_t kI2cAddress3       = 0x44;
constexpr std::uint8_t kI2cAddress4       = 0x45;
// Registers with reference to the INA219 driver code
constexpr std::uint8_t kCurrentReg = 0x04;

class LowPowerCurrent {
 public:
  static std::optional<LowPowerCurrent> create(core::ILogger &logger,
                                               io::II2c &i2c,
                                               const std::uint8_t channel);

  ~LowPowerCurrent();

  std::optional<core::Float> readCurrent();
  std::uint8_t getChannel() const;

 private:
  LowPowerCurrent(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);

 private:
  core::ILogger &logger_;
  io::II2c &i2c_;
  const std::uint8_t channel_;
};
}  // namespace hyped::sensors