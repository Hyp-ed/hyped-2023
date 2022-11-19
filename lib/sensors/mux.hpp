#pragma once

#include "i2c_sensors.hpp"

#include <array>
#include <memory>

#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kDefaultMuxAddress = 0x70;
static constexpr std::uint8_t kMaxNumChannels    = 8;
static constexpr core::Float kFailureThreshold
  = 0.25;  // TODOLater: finalize this value with Electronics

template<class T, std::uint8_t N>
class Mux {
 public:
  Mux(io::I2c &i2c,
      const std::uint8_t mux_address,
      const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors,
      core::ILogger &log);
  ~Mux();

  std::optional<std::array<T, N>> readAllChannels();

 private:
  core::Result selectChannel(const std::uint8_t channel);
  core::Result closeAllChannels();
  core::ILogger &log_;
  io::I2c i2c_;
  const std::uint8_t max_num_unusable_sensors_;
  const std::uint8_t mux_address_;
  const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors_;
};

}  // namespace hyped::sensors