#pragma once

#include <sensors/mux_sensors.hpp>

namespace hyped::utils {

class DummyAdcMuxSensor : public sensors::IMuxSensor<core::Float> {
 public:
  DummyAdcMuxSensor();
  virtual core::Result configure();
  virtual std::optional<core::Float> read();
  virtual std::uint8_t getChannel() const;
};

}  // namespace hyped::utils