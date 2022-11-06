#include "mux.hpp"

namespace hyped::sensors {
template<typename T, std::size_t size>
Mux<T, size>::Mux(hyped::io::I2c &i2c,
                  const uint8_t mux_address,
                  const std::array<std::unique_ptr<II2cSensor<T>>, size> sensors,
                  hyped::core::ILogger &log)
    : log_(log),
      i2c_(i2c),
      sensors_(std::move(sensors))
{
}

template<typename T, std::size_t size>
Mux<T, size>::~Mux()
{
}

template<typename T, std::size_t size>
MuxOperationResult Mux<T, size>::selectMode(const Mode io_mode)
{
  // TODOLater: select mode
  log_.log(hyped::core::LogLevel::kFatal, "Mux select mode not implemented");
  return MuxOperationResult::kFailure;
}

template<typename T, std::size_t size>
MuxOperationResult Mux<T, size>::selectChannel(const uint8_t channel)
{
  // TODOLater: select channel
  log_.log(hyped::core::LogLevel::kFatal, "Mux select channel not implemented");
  return MuxOperationResult::kFailure;
}

template<typename T, std::size_t size>
MuxOperationResult Mux<T, size>::write(const uint8_t data)
{
  // TODOLater: write to mux
  log_.log(hyped::core::LogLevel::kFatal, "Mux write not implemented");
  return MuxOperationResult::kFailure;
}

template<typename T, std::size_t size>
std::optional<std::array<T, size>> Mux<T, size>::readAll()
{
  // TODOLater: read from mux
  log_.log(hyped::core::LogLevel::kFatal, "Mux read not implemented");
  return std::nullopt;
}

}  // namespace hyped::sensors