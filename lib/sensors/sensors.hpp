#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>

namespace hyped::sensors {

enum class WriteResult { kFailure, kSuccess };

/**
 * If a sensor is to be used with an I2C mux, it must inherit from this abstract class.
 */
class II2cSensor {
 public:
  /*
   * This function carries out the initilization steps for a particular sensor.
   */
  virtual bool configure()               = 0;
  virtual WriteResult write()            = 0;
  virtual std::optional<uint16_t> read() = 0;
};

}  // namespace hyped::sensors