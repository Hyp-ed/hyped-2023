#pragma once

#include "gpio.hpp"

#include <unordered_map>

#include <core/types.hpp>

namespace hyped::io {

// GPIO hardware specified addresses and sizes for read, clear, set, size, etc.
static constexpr std::uint32_t pin_size  = 0x1000;
static constexpr std::uint32_t pin_read  = 0x138;
static constexpr std::uint32_t pin_clear = 0x190;
static constexpr std::uint32_t pin_set   = 0x194;

class HardwareGpioReader : public IGpioReader {
 public:
  /**
   * @brief Read a high or low from the GPIO pin.
   */
  virtual std::optional<core::DigitalSignal> read();

 private:
  HardwareGpioReader(const std::uint8_t pin, volatile std::uint32_t *read)
      : pin_map(pin),
        gpio_readAddr(read){};

  const std::uint8_t pin_map;
  volatile std::uint32_t *gpio_readAddr;

  friend class HardwareGpio;
};

class HardwareGpioWriter : public IGpioWriter {
 public:
  /**
   * @brief Writes a high or low to the GPIO pin.
   * @param state The digital signal to write to the pin.
   */
  virtual core::Result write(const core::DigitalSignal state);

 private:
  HardwareGpioWriter(const std::uint8_t pin,
                     volatile std::uint32_t *set,
                     volatile std::uint32_t *clear)
      : pin_map(pin),
        gpio_setAddr(set),
        gpio_clearAddr(clear){};

  const std::uint8_t pin_map;
  volatile std::uint32_t *gpio_setAddr;
  volatile std::uint32_t *gpio_clearAddr;

  friend class HardwareGpio;
};

/**
 * Hardware GPIO interface, requires physical GPIO pins to be present. This should only
 * be instantiated at the top level and then provided to users through the IGpio interface.
 * Ensure inputted pins are defined as pin = 32*X + Y (GPIOX_Y)
 */
class HardwareGpio {
 public:
  HardwareGpio(core::ILogger &log);

  virtual std::optional<std::shared_ptr<IGpioReader>> getReader(const std::uint8_t pin);
  virtual std::optional<std::shared_ptr<IGpioWriter>> getWriter(const std::uint8_t pin);

 private:
  core::ILogger &log_;
  // Bank Addresses are header base addresses.
  // Page 211-213 Figure 6-7/8 P8 Header Pins Beaglebone Bible

  const std::array<off_t, 4> bank_addresses = {0x44e07000, 0x4804c000, 0x481ac000, 0x481ae000};
  std::unordered_map<std::uint8_t, std::shared_ptr<IGpioWriter>> initialized_writers_;
  std::unordered_map<std::uint8_t, std::shared_ptr<IGpioReader>> initialized_readers_;
  // Also hardware specified addresses and sizes for read, clear, set, size, etc.
};

}  // namespace hyped::io