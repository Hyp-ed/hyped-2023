#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

// Four possible device addresses, as per the datasheet
// 0x40 has been chosen as the default
constexpr std::uint8_t kDefaultLowPowerCurrectAddress = 0x40;
constexpr std::uint8_t kLowPowerCurrectAddress2       = 0x41;
constexpr std::uint8_t kLowPowerCurrectAddress3       = 0x44;
constexpr std::uint8_t kLowPowerCurrectAddress4       = 0x45;
// Registers with reference to the INA219 driver code
constexpr std::uint8_t kLowPowerCurrectRegister = 0x04;

class LowPowerCurrent {
 public:
  LowPowerCurrent(core::ILogger &logger,
                  std::shared_ptr<io::II2c> i2c,
                  const std::uint8_t device_address);

  ~LowPowerCurrent();

  std::optional<core::Float> readCurrent();
  std::uint8_t getDeviceAddress() const;

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::II2c> i2c_;
  const std::uint8_t device_address_;
};

}  // namespace hyped::sensors