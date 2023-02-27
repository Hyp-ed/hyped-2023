#pragma once
#include <cstdint>
#include <memory>
#include <optional>

#include <core/types.hpp>
#include <io/gpio.hpp>

class BrakeClass {
 public:
  BrakeClass(const std::uint8_t pin_, const hyped::io::HardwareGpio gpio);
  void stop();
  void release();

 private:
  const std::uint8_t pin;
  const hyped::io::IGpioWriter writer;
}