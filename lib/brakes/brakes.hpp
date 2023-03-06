#include <core/types.hpp>
#include <io/gpio.hpp>

// This class deals with stopping and release brakes of the pod.

class Brakes {
 public:
  Brakes(const std::uint8_t pin_, const io::IGpio gpio);
  void stop();
  void release();

 private:
  const std::uint8_t pin_;
  io::IGpioWriter writer;
}