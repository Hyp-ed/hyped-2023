#include "brakes.hpp"

brakes::BrakeClass(const std::uint8_t pin_, const hyped::io::HardwareGpio gpio)
{
  pin    = pin_;
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