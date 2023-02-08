#pragma once

#include <cstdint>
#include <optional>

#include "io/i2c.hpp"
#include <core/logger.hpp>

namespace hyped::sensors {

// TODOLater: Implement this properly
/**
 * @brief ADC (Texas Instruments) to be used for the hall-effect sensors (Farnell)
 * @details This class is used to convert the analog signal from the hall-effect sensors to a
 * digital signal over I2c
 */
class I2cConverter {
 public:
  static std::optional<I2cConverter> create(core::ILogger &logger,
                                            const std::uint8_t device_address);
  ~I2cConverter();

  std::uint8_t getAddress();

 private:
  I2cConverter(const std::uint8_t device_address);

 private:
  core::ILogger &logger_;
  std::uint8_t address_;
};

}  // namespace hyped::sensors