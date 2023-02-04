#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/gpio.hpp>

namespace hyped::sensors {
class Keyence {
 public:
  /**
   * @brief Configures and creates an instance of Keyence
   */
  static std::optional<Keyence> create(core::ILogger &logger,
                                       io::IGpio &gpio,
                                       std::uint8_t new_pin);
  ~Keyence();

  std::uint8_t getStripeCount();

  /**
   * @brief Will increment stripe_count_ when called if a
   * stripe is detected.
   */
  void updateStripeCount();

 private:
  Keyence(core::ILogger &logger, std::shared_ptr<io::IGpioReader> gpio_reader);

  std::uint8_t pin_;
  std::uint8_t stripe_count_;
  std::shared_ptr<io::IGpioReader> gpio_reader_;
  core::ILogger &logger_;
};

}  // namespace hyped::sensors