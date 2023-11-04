#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/gpio.hpp>
#include <io/hardware_gpio.hpp>

namespace hyped::sensors {
class Keyence {
 public:
  static std::optional<std::shared_ptr<Keyence>> create(core::ILogger &logger,
                                                        std::shared_ptr<io::HardwareGpio> gpio,
                                                        const std::uint8_t new_pin);
  ~Keyence();

  std::uint8_t getStripeCount();

  void updateStripeCount();

  Keyence(core::ILogger &logger, std::shared_ptr<io::IGpioReader> gpio_reader);

 private:
  std::uint8_t pin_;
  std::uint8_t stripe_count_;
  std::shared_ptr<io::IGpioReader> gpio_reader_;
  core::ILogger &logger_;
};

}  // namespace hyped::sensors