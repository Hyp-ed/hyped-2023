#pragma once

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <io/hardware_gpio.hpp>

namespace hyped::sensors {
class StripeSensor {
 public:
  StripeSensor(hyped::core::ILogger &log, const std::uint8_t newPin);
  ~StripeSensor();

  int getStripeCount();
  void updateStripes();

 private:
  std::uint8_t pin;
  int stripeCount = 0;
  hyped::core::ILogger &log_;
};
}  // namespace hyped::sensors
