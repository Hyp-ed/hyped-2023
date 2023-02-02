#include "stripe_sensor.hpp"

namespace hyped::sensors {

StripeSensor::StripeSensor(hyped::core::ILogger &log, const std::uint8_t newPin)
    : pin(newPin),
      log_(log)
{
  hyped::io::HardwareGpio keyence(&log_);
  keyence.getReader(newPin);
};

StripeSensor::~StripeSensor()
{
}

int StripeSensor::getStripeCount()
{
  return stripeCount;
}

void StripeSensor::updateStripes()
{
  if (keyence.read() == core::DigitalSignal::kHigh) { stripeCount++; };
}

}  // namespace hyped::sensors
