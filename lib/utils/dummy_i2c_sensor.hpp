#pragma once

#include <sensors/mux_sensors.hpp>

namespace hyped::utils {

class DummyI2cSensor : public sensors::IMuxSensor<std::uint8_t> {
 public:
  DummyI2cSensor();
  virtual core::Result configure();
  virtual std::optional<std::uint8_t> read();
  virtual std::uint8_t getChannel() const;
};

}  // namespace hyped::utils
