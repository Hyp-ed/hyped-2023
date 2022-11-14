#include "mux.hpp"

namespace hyped::sensors {
template<typename T, std::uint8_t N>
Mux<T, N>::Mux(io::I2c &i2c,
               const std::uint8_t mux_address,
               const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors,
               core::ILogger &log)
    : log_(log),
      i2c_(i2c),
      sensors_(std::move(sensors))
{
  static_assert(N <= 8, "Mux can only have up to 8 channels");
}

template<typename T, std::uint8_t N>
Mux<T, N>::~Mux()
{
}

template<typename T, std::uint8_t N>
core::Result Mux<T, N>::selectChannel(const std::uint8_t channel)
{
  if (channel >= kMaxNumChannels) {
    log_.log(core::LogLevel::kFatal, "Mux Channel number %d is not selectable", channel);
    return core::Result::kFailure;
  }
  const std::uint8_t channel_buffer = 1 << channel;
  const auto i2c_write_result       = i2c_.writeByte(mux_address_, channel_buffer);
  if (i2c_write_result == core::Result::kSuccess) {
    log_.log(core::LogLevel::kInfo, "Mux Channel %d selected", channel);
    return core::Result::kSuccess;
  } else {
    log_.log(core::LogLevel::kFatal, "Failed to select mux channel %d", channel);
    return core::Result::kFailure;
  }
}

template<typename T, std::uint8_t N>
core::Result Mux<T, N>::closeAllChannels()
{
  const std::uint8_t clear_channel_buffer = 0x00;
  const auto i2c_write_result             = i2c_.writeByte(mux_address_, clear_channel_buffer);
  if (i2c_write_result == core::Result::kSuccess) {
    log_.log(core::LogLevel::kInfo, "All mux channels closed");
    return core::Result::kSuccess;
  } else {
    log_.log(core::LogLevel::kFatal, "Failed to close all mux channels");
    return core::Result::kFailure;
  }
}

template<typename T, std::uint8_t N>
std::optional<std::array<T, N>> Mux<T, N>::readAllChannels()
{
  std::array<T, N> mux_data;
  for (std::uint8_t i = 0; i < N; ++i) {
    const auto sensor          = sensors_[i];
    const std::uint8_t channel = sensor->getChannel();
    // First ensure correct channel is selected
    const auto channel_select_result = selectChannel(channel);
    if (channel_select_result == core::Result::kFailure) {
      log_.log(core::LogLevel::kFatal, "Failed to select mux channel %d", channel);
      return std::nullopt;
    }
    // Next ensure sensor is configured for operation
    const auto configure_result = sensor->configure();
    if (configure_result == core::Result::kFailure) {
      log_.log(core::LogLevel::kFatal, "Failed to configure sensor at mux channel %d", channel);
      return std::nullopt;
    }
    // Finally read sensor data
    const std::optional<T> sensor_data = sensor->read();
    if (sensor_data.has_value()) {
      mux_data.at(i) = sensor_data.value();
    } else {
      log_.log(core::LogLevel::kFatal, "Failed to get mux data from channel %d", channel);
      return std::nullopt;
    }
    const auto closing_channel_result = closeAllChannels();
    if (closing_channel_result == core::Result::kFailure) {
      log_.log(core::LogLevel::kFatal, "Failed to close all mux channels while reading");
      return std::nullopt;
    }
  }
  return mux_data;
}

}  // namespace hyped::sensors