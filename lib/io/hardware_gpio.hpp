#pragma once

#include "gpio.hpp"

#include <core/types.hpp>
#include <unordered_map>

namespace hyped::io {

class HardwareGpioReader : public IGpioReader {
 public:
  virtual std::optional<core::DigitalSignal> read();

 private:
  HardwareGpioReader(uint8_t pin, volatile unsigned int *read): pinMAP(pin), gpio_readAddr(read){};

  const uint8_t pinMAP;
  volatile unsigned int *gpio_readAddr;

  friend class HardwareGpio;
};







class HardwareGpioWriter : public IGpioWriter {
 public:
  virtual GpioWriteResult write(const core::DigitalSignal state);

 private:
  HardwareGpioWriter(const uint8_t pin,  volatile unsigned int *set, volatile unsigned int *clear): pinMAP(pin), gpio_setAddr(set), gpio_clearAddr(clear){};
  
  const uint8_t pinMAP;
  volatile unsigned int *gpio_setAddr;
  volatile unsigned int *gpio_clearAddr;

  friend class HardwareGpio;
};

/**
 * Hardware GPIO interface, requires physical GPIO pins to be present. This should only
 * be instantiated at the top level and then provided to users through the IGpio interface.
 */

//GPIO  Start Addr   End Addr
//GPIO0 0x44E0_7000  0x44E0_7FFF
//GPIO1 0x4804_C000  0x4804_CFFF
//GPIO2 0x481A_C000  0x481A_CFFF
//GPIO3 0x481A_E000  0x481A_EFFF

//GPIO_DATAIN (READ) 0x138h
//GPIO_DATAOUT 0x13c
//SET 0x194
//CLEAR 0x190
//  const uint8_t bank   = pin_ / 32;  // offset: GPIO_0,1,2,3
//  const uint8_t pin_id = pin_ % 32;


class HardwareGpio {
 public:
  HardwareGpio(hyped::core::ILogger &log);

  virtual std::optional<std::shared_ptr<IGpioReader>> getReader(const std::uint8_t pin);
  virtual std::optional<std::shared_ptr<IGpioWriter>> getWriter(const std::uint8_t pin);

 private:
  hyped::core::ILogger &log_;
  const off_t bankAddresses[4] = {0x44e07000, 0x4804c000, 0x481ac000, 0x481ae000};
  std::unordered_map<uint8_t, std::shared_ptr<IGpioWriter>> InitializedWriters;
  std::unordered_map<uint8_t, std::shared_ptr<IGpioReader>> InitializedReaders;
  
  static constexpr unsigned int pinSize     = 0x1000;
  static constexpr unsigned int pinRead     = 0x138;
  static constexpr unsigned int pinClear    = 0x190;
  static constexpr unsigned int pinSet      = 0x194;
};

}  // namespace hyped::io
