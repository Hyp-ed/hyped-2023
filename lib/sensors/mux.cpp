#include "mux.hpp"

namespace hyped::sensors {
template<typename T, std::size_t N>
Mux<T, N>::Mux(io::I2c &i2c,
               const std::uint8_t mux_address,
               const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors,
               core::ILogger &log)
    : log_(log),
      i2c_(i2c),
      sensors_(std::move(sensors))
{
}

template<typename T, std::size_t N>
Mux<T, N>::~Mux()
{
}

template<typename T, std::size_t N>
core::Result Mux<T, N>::selectChannel(const std::uint8_t channel)
{
  const std::uint8_t channel_buffer = 1 << channel;
  const auto i2c_write_result       = i2c_.writeByte(mux_address_, channel_buffer);
  if (i2c_write_result == core::Result::kSuccess) {
    log_.log(core::LogLevel::kInfo, "Mux : Channel %d selected", channel);
    return core::Result::kSuccess;
  } else {
    log_.log(core::LogLevel::kFatal, "Mux : Failed to select channel %d", channel);
    return core::Result::kFailure;
  }
}

template<typename T, std::size_t N>
core::Result Mux<T, N>::closeAllChannels()
{
  const std::uint8_t clear_channel_buffer = 0x00;
  const auto i2c_write_result             = i2c_.writeByte(mux_address_, clear_channel_buffer);
  if (i2c_write_result == core::Result::kSuccess) {
    log_.log(core::LogLevel::kInfo, "Mux : All channels closed");
    return core::Result::kSuccess;
  } else {
    log_.log(core::LogLevel::kFatal, "Mux : Failed to close all channels");
    return core::Result::kFailure;
  }
}

template<typename T, std::size_t N>
std::optional<std::array<T, N>> Mux<T, N>::readAllChannels()
{
  std::array<T, N> mux_data;
  for (std::size_t i = 0; i < N; ++i) {
    const auto sensor          = sensors_[i];
    const std::uint8_t channel = sensor->getChannel();
    // First ensure correct channel is selected
    const auto channel_select_result = selectChannel(channel);
    if (channel_select_result == core::Result::kFailure) {
      log_.log(core::LogLevel::kFatal, "Mux : Failed to select channel %d", channel);
      return std::nullopt;
    }
    // Next ensure sensor is configured for operation
    const auto configure_result = sensor->configure();
    if (configure_result == core::Result::kFailure) {
      log_.log(core::LogLevel::kFatal, "Mux : Failed to configure sensor at channel %d", channel);
      return std::nullopt;
    }
    // Finally read sensor data
    const std::optional<T> sensor_data = sensor->read();
    if (sensor_data.has_value()) {
      mux_data.at(i) = sensor_data.value();
    } else {
      log_.log(core::LogLevel::kFatal, "Mux : Failed to get data from channel %d", channel);
      return std::nullopt;
    }
    const auto closing_channel_result = closeAllChannels();
    if (closing_channel_result == core::Result::kFailure) {
      log_.log(core::LogLevel::kFatal, "Mux : Failed to close all channels while reading");
      return std::nullopt;
    }
  }
  return mux_data;
}

}  // namespace hyped::sensors