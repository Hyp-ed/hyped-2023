#pragma once

#include "i2c_sensors.hpp"

#include <array>
#include <memory>

#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kDefaultMuxAddress = 0x70;

template<class T, std::size_t N>
class Mux {
 public:
  Mux(hyped::io::I2c &i2c,
      const std::uint8_t mux_address,
      const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors,
      hyped::core::ILogger &log);
  ~Mux();

  std::optional<std::array<T, N>> readAllChannels();

 private:
  hyped::core::Result selectChannel(const std::uint8_t channel);
  hyped::core::Result closeAllChannels();
  hyped::core::ILogger &log_;
  hyped::io::I2c i2c_;
  const std::uint8_t mux_address_;
  const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors_;
};

}  // namespace hyped::sensors