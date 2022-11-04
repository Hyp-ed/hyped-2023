#pragma once

#include "sensors.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>
#include <vector>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

enum class Mode { kWrite = 0, kRead };
enum class MuxWriteResult { kFailure = 0, kSuccess };

template<class T>
class Mux {
 public:
  Mux(hyped::io::I2c &i2c,
      const uint8_t mux_address,
      II2cSensor<T> &sensor,
      hyped::core::ILogger &log);
  ~Mux();

  bool selectMode(Mode io_mode);
  MuxWriteResult write(uint8_t data);
  std::optional<std::vector<T>> readAll();
  bool selecChannel(uint8_t channel);

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c i2c_;
};

}  // namespace hyped::sensors