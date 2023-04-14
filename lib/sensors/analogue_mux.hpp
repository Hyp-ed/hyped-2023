#pragma once

#include "mux_sensors.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/gpio.hpp>

namespace hyped::sensors {

constexpr std::uint8_t kNumSelectorPins = 4;

/**
 * @brief   Class for analogue mux where channel selection is done by GPIO pins
 * @details 16 channels are available, selection is done by 4 GPIO pins (think 4x16 decoder)
 */
template<class T, std::uint8_t N>
class AnalogueMux {
 public:
  /**
   * @brief   Creates an AnalogueMux object with specific selector pins and disable input pin
   * @details The disable input pin is just the active low enable input pin (from datasheet)
   * @param   selector_pin_writers  Array of 4 GPIO pin writers in the order s0, s1, s2 and s3
   * @param   disable_input_writer     GPIO pin corresponding to E bar (active low enable input pin)
   */
  static std::optional<std::shared_ptr<AnalogueMux<T, N>>> create(
    core::ILogger &logger,
    std::array<std::shared_ptr<io::IGpioWriter>, kNumSelectorPins> selector_pin_writers,
    std::shared_ptr<io::IGpioWriter> disable_input_writer,
    std::array<std::unique_ptr<IMuxSensor<T>>, N> &sensors);
  ~AnalogueMux();

  std::optional<std::array<T, N>> readAllChannels();

 private:
  AnalogueMux(core::ILogger &logger,
              std::array<std::shared_ptr<io::IGpioWriter>, kNumSelectorPins> selector_pins,
              std::shared_ptr<io::IGpioWriter> disable_input_writer,
              std::array<std::unique_ptr<IMuxSensor<T>>, N> &sensors);
  core::Result selectChannel(const std::uint8_t channel);
  core::Result closeAllChannels();

 private:
  core::ILogger &logger_;
  const std::array<std::shared_ptr<io::IGpioWriter>, kNumSelectorPins> selector_pin_writers_;
  cosnt std::shared_ptr<io::IGpioWriter> disable_input_writer_;
  const std::array<std::unique_ptr<IMuxSensor<T>>, N> sensors_;
};

template<typename T, std::uint8_t N>
std::optional<std::shared_ptr<AnalogueMux<T, N>>> AnalogueMux<T, N>::create(
  core::ILogger &logger,
  std::array<std::shared_ptr<io::IGpioWriter>, kNumSelectorPins> selector_pin_writers,
  std::shared_ptr<io::IGpioWriter> disable_input_writer,
  std::array<std::unique_ptr<IMuxSensor<T>>, N> &sensors)
{
  if (N > 16) {
    logger.log(core::LogLevel::kFatal,
               "Failed to create AnalogueMux instance, maximum 16 channels only");
    return std::nullopt;
  }
  return std::make_shared<AnalogueMux<T, N>>(logger, selector_pin_writers, sensors);
}

template<typename T, std::uint8_t N>
AnalogueMux<T, N>::AnalogueMux(
  core::ILogger &logger,
  std::array<std::shared_ptr<io::IGpioWriter>, kNumSelectorPins> selector_pin_writers,
  std::shared_ptr<io::IGpioWriter> disable_input_writer,
  std::array<std::unique_ptr<IMuxSensor<T>>, N> &sensors)
    : logger_(logger),
      selector_pin_writers_(std::move(selector_pin_writers)),
      sensors_(std::move(sensors))
{
}

template<typename T, std::uint8_t N>
AnalogueMux<T, N>::~AnalogueMux()
{
}

template<typename T, std::uint8_t N>
std::optional<std::array<T, N>> AnalogueMux<T, N>::readAllChannels()
{
  std::array<T, N> values;
  for (std::size_t i = 0; i < N; ++i) {
    core::Result channel_select_result = selectChannel(i);
    if (channel_select_result == core::Result::kFailure) {
      logger_.log(core::LogLevel::kError, "Failed to select channel %d for analogue mux", i);
      return std::nullopt;
    }
    std::optional<T> sensor_value = sensors_[i]->read();
    if (!sensor_value) {
      logger_.log(
        core::LogLevel::kError, "Failed to read from sensor on channel %d on analogue mux", i);
      return std::nullopt;
    }
    values[i] = *sensor_value;
  }
  return values;
}

template<typename T, std::uint8_t N>
core::Result AnalogueMux<T, N>::selectChannel(const std::uint8_t channel)
{
  std::bitset binary_selector = std::bitset<4>(channel);
  for (std::size_t i = 0; i < kNumSelectorPins; ++i) {
    core::Result write_result = selector_pin_writers_[i]->write(
      binary_selector.test(i) ? core::DigitalSignal::kHigh : core::DigitalSignal::kLow);
    if (write_result == core::Result::kFailure) {
      logger_.log(
        core::LogLevel::kError, "Failed to write to selector GPIO pin %d for analogue mux", i);
      return core::Result::kFailure;
    }
  }
  return core::Result::kSuccess;
}

template<typename T, std::uint8_t N>
core::Result AnalogueMux<T, N>::closeAllChannels()
{
  core::Result write_result = disable_input_writer_->write(core::DigitalSignal::kHigh);
  if (write_result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal, "Failed to disable analogue mux");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

}  // namespace hyped::sensors
