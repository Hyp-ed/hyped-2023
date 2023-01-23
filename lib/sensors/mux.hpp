#pragma once

#include "i2c_sensors.hpp"

#include <array>
#include <memory>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kDefaultMuxAddress = 0x70;
static constexpr std::uint8_t kMaxNumChannels    = 8;
static constexpr core::Float kFailureThreshold
  = 0.25;  // TODOLater: finalize this value with Electronics

template<class T, std::uint8_t N>
class Mux {
 public:
  Mux(core::ILogger &logger,
      io::II2c &i2c,
      const std::uint8_t mux_address,
      std::array<std::unique_ptr<II2cMuxSensor<T>>, N> &sensors);
  ~Mux();

  std::optional<std::array<T, N>> readAllChannels();

 private:
  core::Result selectChannel(const std::uint8_t channel);
  core::Result closeAllChannels();

  core::ILogger &logger_;
  io::II2c &i2c_;
  const std::uint8_t mux_address_;
  const std::array<std::unique_ptr<II2cMuxSensor<T>>, N> sensors_;
  const std::uint8_t max_num_unusable_sensors_;
};

template<typename T, std::uint8_t N>
Mux<T, N>::Mux(core::ILogger &logger,
               io::II2c &i2c,
               const std::uint8_t mux_address,
               std::array<std::unique_ptr<II2cMuxSensor<T>>, N> &sensors)
    : logger_(logger),
      i2c_(i2c),
      mux_address_(mux_address),
      sensors_(std::move(sensors)),
      max_num_unusable_sensors_(static_cast<std::uint8_t>(kFailureThreshold * N))
{
  static_assert(N <= 8, "Mux can only have up to 8 channels");
}

template<typename T, std::uint8_t N>
Mux<T, N>::~Mux()
{
}

template<typename T, std::uint8_t N>
std::optional<std::array<T, N>> Mux<T, N>::readAllChannels()
{
  // Zero-initialize the array
  std::array<T, N> mux_data{};
  std::uint8_t num_unusable_sensors = 0;
  for (std::uint8_t i = 0; i < N; ++i) {
    const auto &sensor         = sensors_.at(i);
    const std::uint8_t channel = sensor->getChannel();
    // First ensure correct channel is selected
    const auto channel_select_result = selectChannel(channel);
    if (channel_select_result == core::Result::kFailure) {
      logger_.log(core::LogLevel::kFatal, "Failed to select mux channel %d", channel);
      return std::nullopt;
    }
    // Next ensure sensor is configured for operation
    const auto configure_result = sensor->configure();
    if (configure_result == core::Result::kFailure) {
      logger_.log(core::LogLevel::kFatal, "Failed to configure sensor at mux channel %d", channel);
      ++num_unusable_sensors;
      continue;
    }
    // Finally read sensor data
    const auto sensor_data = sensor->read();
    if (!sensor_data) {
      logger_.log(core::LogLevel::kFatal, "Failed to get mux data from channel %d", channel);
      ++num_unusable_sensors;
      continue;
    }
    mux_data.at(i)                    = sensor_data.value();
    const auto closing_channel_result = closeAllChannels();
    if (closing_channel_result == core::Result::kFailure) {
      logger_.log(core::LogLevel::kFatal, "Failed to close all mux channels while reading");
      return std::nullopt;
    }
  }
  // If too many sensors are unusable, assume mux is faulty
  if (num_unusable_sensors > max_num_unusable_sensors_) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to read from more than %0.f%% of sensors on the mux",
                kFailureThreshold * 100);
    return std::nullopt;
  }
  return mux_data;
}

template<typename T, std::uint8_t N>
core::Result Mux<T, N>::selectChannel(const std::uint8_t channel)
{
  if (channel >= kMaxNumChannels) {
    logger_.log(core::LogLevel::kFatal, "Mux Channel number %d is not selectable", channel);
    return core::Result::kFailure;
  }
  const std::uint8_t channel_buffer = 1 << channel;
  const auto i2c_write_result       = i2c_.writeByte(mux_address_, channel_buffer);
  if (i2c_write_result == core::Result::kSuccess) {
    logger_.log(core::LogLevel::kInfo, "Mux Channel %d selected", channel);
    return core::Result::kSuccess;
  } else {
    logger_.log(core::LogLevel::kFatal, "Failed to select mux channel %d", channel);
    return core::Result::kFailure;
  }
}

template<typename T, std::uint8_t N>
core::Result Mux<T, N>::closeAllChannels()
{
  const std::uint8_t clear_channel_buffer = 0x00;
  const auto i2c_write_result             = i2c_.writeByte(mux_address_, clear_channel_buffer);
  if (i2c_write_result == core::Result::kSuccess) {
    logger_.log(core::LogLevel::kInfo, "All mux channels closed");
    return core::Result::kSuccess;
  } else {
    logger_.log(core::LogLevel::kFatal, "Failed to close all mux channels");
    return core::Result::kFailure;
  }
}

}  // namespace hyped::sensors
