#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/adc.hpp>

namespace hyped::sensors {

// Skeleton code for the thermistor class
// TODOLater: Implement this properly
class Thermistor {
 public:
  static std::optional<Thermistor> create(core::ILogger &logger,
                                          std::shared_ptr<io::IAdc> adc,
                                          const std::uint8_t new_pin);
  ~Thermistor();

  std::uint16_t getTemperature();

 private:
  Thermistor(core::ILogger &logger, std::shared_ptr<io::IAdc> adc, const std::uint8_t new_pin);

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::IAdc> adc_;
  std::uint8_t pin_;
};

}  // namespace hyped::sensors