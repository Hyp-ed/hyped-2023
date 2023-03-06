#include "brakes.hpp"

brakes::Brakes(const std::uint8_t pin, const io::IGpio gpio) : pin_(pin)
{
  writer = gpio.getWriter(pin);
}

void brakes::stop()
{
  writer.write(core::DigitalSignal::kHigh);
}

void brakes::release()
{
  writer.write(core::DigitalSignal::kLow);
}