#pragma once

#include "i2c_sensors.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

enum class Mode { kWrite = 0, kRead };
enum class MuxOperationResult { kFailure = 0, kSuccess };

template<class T, std::size_t size>
class Mux {
 public:
  Mux(hyped::io::I2c &i2c,
      const uint8_t mux_address,
      const std::array<std::unique_ptr<II2cSensor<T>>, size> sensors,
      hyped::core::ILogger &log);
  ~Mux();

  MuxOperationResult selectMode(const Mode io_mode);
  MuxOperationResult write(const uint8_t data);
  std::optional<std::array<T, size>> readAll();
  MuxOperationResult selectChannel(const uint8_t channel);

 private:
  hyped::core::ILogger &log_;
  hyped::io::I2c i2c_;
  const std::array<std::unique_ptr<II2cSensor<T>>, size> sensors_;
};

}  // namespace hyped::sensors