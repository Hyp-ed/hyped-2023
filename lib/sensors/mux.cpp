#include "mux.hpp"

namespace hyped::sensors {
template<typename T, std::size_t size>
Mux<T, size>::Mux(hyped::io::I2c &i2c,
                  const std::uint8_t mux_address,
                  const std::array<std::unique_ptr<II2cMuxSensor<T>>, size> sensors,
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
hyped::core::Result Mux<T, size>::selectChannel(const std::uint8_t channel)
{
  const std::uint8_t channel_buffer          = 1 << channel;
  const hyped::core::Result i2c_write_result = i2c_.writeByte(mux_address_, channel_buffer);
  if (i2c_write_result == hyped::core::Result::kSuccess) {
    log_.log(hyped::core::LogLevel::kInfo, "Mux : Channel %d selected", channel);
    return hyped::core::Result::kSuccess;
  } else {
    log_.log(hyped::core::LogLevel::kFatal, "Mux : Failed to select channel %d", channel);
    return hyped::core::Result::kFailure;
  }
}

template<typename T, std::size_t size>
hyped::core::Result Mux<T, size>::closeAllChannels()
{
  const std::uint8_t clear_channel_buffer    = 0x00;
  const hyped::core::Result i2c_write_result = i2c_.writeByte(mux_address_, clear_channel_buffer);
  if (i2c_write_result == hyped::core::Result::kSuccess) {
    log_.log(hyped::core::LogLevel::kInfo, "Mux : All channels closed");
    return hyped::core::Result::kSuccess;
  } else {
    log_.log(hyped::core::LogLevel::kFatal, "Mux : Failed to close all channels");
    return hyped::core::Result::kFailure;
  }
}

template<typename T, std::size_t size>
std::optional<std::array<T, size>> Mux<T, size>::readAllChannels()
{
  std::array<T, size> mux_data;
  for (std::size_t i = 0; i < size; ++i) {
    const auto sensor          = sensors_[i];
    const std::uint8_t channel = sensor->getChannel();
    // First ensure correct channel is selected
    const hyped::core::Result channel_select_result = selectChannel(channel);
    if (channel_select_result == hyped::core::Result::kFailure) {
      log_.log(hyped::core::LogLevel::kFatal, "Mux : Failed to select channel %d", channel);
      return std::nullopt;
    }
    // Next ensure sensor is configured for operation
    const auto configure_result = sensor->configure();
    if (configure_result == hyped::core::Result::kFailure) {
      log_.log(
        hyped::core::LogLevel::kFatal, "Mux : Failed to configure sensor at channel %d", channel);
      return std::nullopt;
    }
    // Finally read sensor data
    const std::optional<T> sensor_data = sensor->read();
    if (sensor_data.has_value()) {
      mux_data.at(i) = sensor_data.value();
    } else {
      log_.log(hyped::core::LogLevel::kFatal, "Mux : Failed to get data from channel %d", channel);
      return std::nullopt;
    }
    const hyped::core::Result closing_channel_result = closeAllChannels();
    if (closing_channel_result == hyped::core::Result::kFailure) {
      log_.log(hyped::core::LogLevel::kFatal, "Mux : Failed to close all channels while reading");
      return std::nullopt;
    }
  }
  return mux_data;
}

}  // namespace hyped::sensors