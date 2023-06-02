#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/adc.hpp>
#include <mux_sensors.hpp>

namespace hyped::sensors {

// Skeleton code for the thermistor class
// TODOLater: Implement this properly
class Thermistor : public IMuxSensor<core::Float> {
 public:
  static std::optional<Thermistor> create(core::ILogger &logger,
                                          std::shared_ptr<io::IAdc> adc,
                                          std::uint8_t channel);
  Thermistor(core::ILogger &logger, std::shared_ptr<io::IAdc> adc, std::uint8_t channel);
  ~Thermistor();
  std::uint8_t getChannel() const override;
  std::optional<core::Float> read() override;

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::IAdc> adc_;
  std::uint8_t channel_;
};

}  // namespace hyped::sensors