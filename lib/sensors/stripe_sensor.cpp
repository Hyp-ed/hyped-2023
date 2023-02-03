#include "stripe_sensor.hpp"

namespace hyped::sensors {

Keyence::Keyence(core::ILogger &log, const std::uint8_t newPin) : pin(newPin), logger_(log)
{
  stripe_count_ = 0;
  io::HardwareGpio hardware(logger_);
  keyence = hardware.getReader(pin);
};

Keyence::~Keyence()
{
}

int Keyence::getStripeCount()
{
  return stripe_count_;
}

void Keyence::updateStripes()
{
  if (keyence.read() == core::DigitalSignal::kHigh) { ++stripe_count_; };
}

}  // namespace hyped::sensors
