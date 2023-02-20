#pragma once

#include <cstdio>
#include <optional>

#include <core/types.hpp>

namespace hyped::sensors {

/**
 * If a sensor is to be used with any mux, it must inherit from this abstract class.
 */
template<typename T>
class IMuxSensor {
 public:
  /*
   * This function carries out the initilization steps for a particular sensor.
   */
  virtual std::optional<T> read()         = 0;
  virtual std::uint8_t getChannel() const = 0;
  virtual ~IMuxSensor() {}
};

}  // namespace hyped::sensors