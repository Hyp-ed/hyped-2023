#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/gpio.hpp>

namespace hyped::sensors {
class Keyence {
 public:
  static std::optional<Keyence> create(core::ILogger &log, io::IGpio &gpio, std::uint8_t new_pin);
  ~Keyence();

  std::uint8_t getStripeCount();
  void updateStripeCount();

 private:
  Keyence(core::ILogger &log, std::shared_ptr<io::IGpioReader> gpio_reader);

  std::uint8_t pin_;
  std::uint8_t stripe_count_;
  std::shared_ptr<io::IGpioReader> gpio_reader_;
  core::ILogger &logger_;
};

}  // namespace hyped::sensors
