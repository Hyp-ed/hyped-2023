#include "brakes.hpp"

namespace hyped::brakes {
Brakes::Brakes(const std::uint8_t pin, const io::IGpio gpio) : pin_(pin)
{
  writer_ = gpio.getWriter(pin);
}

void Brakes::stop()
{
  writer_.write(core::DigitalSignal::kHigh);
}

void Brakes::release()
{
  writer_.write(core::DigitalSignal::kLow);
}
}  // namespace hyped::brakes