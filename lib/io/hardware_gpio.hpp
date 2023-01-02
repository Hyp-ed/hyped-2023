#pragma once

#include "gpio.hpp"

#include <string>

#include <core/types.hpp>

// The edge parameter is used to set the interrupt trigger for the pin.
enum class Edge { kNone = 0, kRising, kFalling, kBoth };
enum class Direction { kIn = 0, kOut };

namespace hyped::io {

class HardwareGpioReader : public IGpioReader {
 public:
  /**
   * @brief Reads the digital signal from the GPIO pin.
   * @return The digital signal read from the pin.
   */
  std::optional<core::DigitalSignal> readPin();
  ~HardwareGpioReader();

 private:
  HardwareGpioReader(core::ILogger &logger, const int read_file_descritor);

  core::ILogger &logger_;
  int read_file_descriptor_;
  friend class HardwareGpio;
};

class HardwareGpioWriter : public IGpioWriter {
 public:
  core::Result writeToPin(const core::DigitalSignal state);
  ~HardwareGpioWriter();

 private:
  HardwareGpioWriter(core::ILogger &logger, const int write_file_descriptor);

  core::ILogger &logger_;
  int write_file_descriptor_;
  friend class HardwareGpio;
};

/**
 * Hardware GPIO interface, requires physical GPIO pins to be present. This should only
 * be instantiated at the top level and then provided to users through the IGpio interface.
 * Ensure inputted pin are defined as pin = 32 * X + Y (GPIOX_Y)
 */
class HardwareGpio {
 public:
  HardwareGpio(core::ILogger &logger);

  std::optional<std::shared_ptr<IGpioReader>> getReader(const std::uint8_t pin,
                                                        const Edge edge = Edge::kBoth);
  std::optional<std::shared_ptr<IGpioWriter>> getWriter(const std::uint8_t pin,
                                                        const Edge edge = Edge::kBoth);

 private:
  core::Result initialisePin(const std::uint8_t pin, const Edge edge, const Direction direction);
  core::Result exportPin(const std::uint8_t pin);
  int getFileDescriptor(const std::uint8_t pin, const Direction direction);
  static const std::string getEdgeString(const Edge edge);
  static const std::string getDirectionString(const Direction direction);

  core::ILogger &logger_;
};

}  // namespace hyped::io