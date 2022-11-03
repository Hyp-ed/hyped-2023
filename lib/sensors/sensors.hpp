#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>

namespace hyped::sensors {

/**
 * If a sensor is to be used with an I2C mux, it must inherit from this abstract class.
 */
template <typename T>
class II2cSensor {
 public:
  /*
   * This function carries out the initilization steps for a particular sensor.
   */
  virtual bool configure()        = 0;
  virtual std::optional<T> read() = 0;
};

}  // namespace hyped::sensors