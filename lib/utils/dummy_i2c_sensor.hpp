#pragma once

#include <core/time.hpp>
#include <sensors/i2c_sensors.hpp>

namespace hyped::utils {

class DummyI2cSensor : public sensors::II2cMuxSensor<std::uint8_t> {
 public:
  DummyI2cSensor(core::ITimeSource &time_source);
  virtual core::Result configure();
  virtual std::optional<core::Measurement<std::uint8_t>> read();
  virtual std::uint8_t getChannel() const;

 private:
  core::ITimeSource &time_source_;
};

}  // namespace hyped::utils
