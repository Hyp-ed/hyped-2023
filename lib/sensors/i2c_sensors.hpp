#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>

namespace hyped::sensors {

enum class I2cConfigureResult { kFailure = 0, kSuccess };

/**
 * If a sensor is to be used with an I2C mux, it must inherit from this abstract class.
 */
template<typename T>
class II2cMuxSensor {
 public:
  /*
   * This function carries out the initilization steps for a particular sensor.
   */
  virtual I2cConfigureResult configure() = 0;
  virtual std::optional<T> read()        = 0;
  virtual std::uint8_t getChannel()      = 0;
  virtual ~II2cMuxSensor() {}
};

}  // namespace hyped::sensors