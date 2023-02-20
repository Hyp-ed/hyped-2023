#pragma once

#include "mux_sensors.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/gpio.hpp>

namespace hyped::sensors {

template<class T, std::uint8_t N>
class AnalogueMux {
 public:
  AnalogueMux(core::ILogger &logger,
              std::shared_ptr<io::IGpio> gpio,
              const std::uint8_t mux_address,
              std::array<std::unique_ptr<IMuxSensor<T>>, N> &sensors);
  ~AnalogueMux();

  std::optional<std::array<T, N>> readAllChannels(const std::uint8_t channel);

 private:
  core::Result selectChannel(const std::uint8_t channel);
  core::Result closeAllChannels();

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::IGpioWriter> gpio_writer_;
  const std::uint8_t mux_address_;
  const std::array<std::unique_ptr<IMuxSensor<T>>, N> sensors_;
};

template<typename T, std::uint8_t N>
AnalogueMux<T, N>::AnalogueMux(core::ILogger &logger,
                               std::shared_ptr<io::IGpio> gpio,
                               const std::uint8_t mux_address,
                               std::array<std::unique_ptr<IMuxSensor<T>>, N> &sensors)
    : logger_(logger),
      gpio_writer_(gpio->getWriter()),
      mux_address_(mux_address),
      sensors_(std::move(sensors))
{
}

}  // namespace hyped::sensors
