#include "mux.hpp"

namespace hyped::sensors {
template<typename T>
Mux<T>::Mux(hyped::io::I2c &i2c,
            const uint8_t mux_address,
            std::vector<std::unique_ptr<II2cSensor<T>>> sensors,
            hyped::core::ILogger &log)
    : log_(log),
      i2c_(i2c),
      sensors_(std::move(sensors))
{
}

template<typename T>
Mux<T>::~Mux()
{
}

template<typename T>
bool Mux<T>::selectMode(Mode io_mode)
{
  // TODOLater: select mode
  log_.log(hyped::core::LogLevel::kFatal, "Mux select mode not implemented");
  return false;
}

template<typename T>
MuxWriteResult Mux<T>::write(uint8_t data)
{
  // TODOLater: write to mux
  log_.log(hyped::core::LogLevel::kFatal, "Mux write not implemented");
  return MuxWriteResult::kFailure;
}

template<typename T>
std::optional<std::vector<T>> Mux<T>::readAll()
{
  // TODOLater: read from mux
  log_.log(hyped::core::LogLevel::kFatal, "Mux read not implemented");
  return std::nullopt;
}

}  // namespace hyped::sensors