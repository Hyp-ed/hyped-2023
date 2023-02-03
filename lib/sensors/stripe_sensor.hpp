#pragma once

#include <cstdint>
#include <optional>

#include "io/gpio.hpp"
#include <core/logger.hpp>
#include <io/hardware_gpio.hpp>

namespace hyped::sensors {
class Keyence {
 public:
  Keyence(core::ILogger &log, const std::uint8_t newPin);
  ~Keyence();

  int getStripeCount();
  void updateStripes();

 private:
  std::uint8_t pin;
  uint16_t stripe_count_;
  std::optional<std::shared_ptr<IGpioReader>> keyence;
  hyped::core::ILogger &logger_;
};
}  // namespace hyped::sensors
