#pragma once

#include "gpio.hpp"

#include <unordered_map>

#include <core/types.hpp>

namespace hyped::io {

class HardwareGpioReader : public IGpioReader {
 public:
  virtual std::optional<core::DigitalSignal> read();

 private:
  HardwareGpioReader(std::uint8_t pin, volatile uint32_t *read)
      : pinMAP(pin),
        gpio_readAddr(read){};

  const std::uint8_t pinMAP;
  volatile uint32_t *gpio_readAddr;

  friend class HardwareGpio;
};

class HardwareGpioWriter : public IGpioWriter {
 public:
  virtual core::Result write(const core::DigitalSignal state);

 private:
  HardwareGpioWriter(const std::uint8_t pin, volatile uint32_t *set, volatile uint32_t *clear)
      : pinMAP(pin),
        gpio_setAddr(set),
        gpio_clearAddr(clear){};

  const std::uint8_t pinMAP;
  volatile uint32_t *gpio_setAddr;
  volatile uint32_t *gpio_clearAddr;

  friend class HardwareGpio;
};

class HardwareGpio {
 public:
  HardwareGpio(core::ILogger &log);

  virtual std::optional<std::shared_ptr<IGpioReader>> getReader(const std::uint8_t pin);
  virtual std::optional<std::shared_ptr<IGpioWriter>> getWriter(const std::uint8_t pin);

 private:
  core::ILogger &log_;
  // Bank Addresses are header base addresses.
  // Page 211-213 Figure 6-7/8 P8 Header Pins Beaglebone Bible

  const off_t bankAddresses[4] = {0x44e07000, 0x4804c000, 0x481ac000, 0x481ae000};
  std::unordered_map<std::uint8_t, std::shared_ptr<IGpioWriter>> InitializedWriters;
  std::unordered_map<std::uint8_t, std::shared_ptr<IGpioReader>> InitializedReaders;
  // Also hardware specified addresses and sizes for read, clear, set, size, etc.
  static constexpr uint32_t pinSize  = 0x1000;
  static constexpr uint32_t pinRead  = 0x138;
  static constexpr uint32_t pinClear = 0x190;
  static constexpr uint32_t pinSet   = 0x194;
};

}  // namespace hyped::io
