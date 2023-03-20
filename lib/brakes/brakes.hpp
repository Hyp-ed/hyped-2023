#pragma once

#include <core/types.hpp>
#include <io/gpio.hpp>

// This class deals with stopping and release brakes of the pod.
namespace hyped::brakes {
class Brakes {
 public:
  Brakes(const std::uint8_t pin, const io::IGpio gpio);
  void stop();
  void release();

 private:
  const std::uint8_t pin_;
  io::IGpioWriter writer_;
}
}  // namespace hyped::brakes