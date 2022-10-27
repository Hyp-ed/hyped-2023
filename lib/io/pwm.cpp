#include "pwm.hpp"

namespace hyped::io {

Pwm::Pwm(const std::string device, hyped::core::ILogger &log) : log_(log)
{
  log_.log(hyped::core::LogLevel::kInfo, "Initializing PWM device %s", device.c_str());
}

Pwm::~Pwm()
{
  log_.log(hyped::core::LogLevel::kInfo, "Destroying PWM");
}

void Pwm::setFrequency(uint16_t frequency)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM frequency to %d", frequency);
}

void Pwm::setDutyCycle(uint8_t duty_cycle)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM duty cycle to %d", duty_cycle);
}

void Pwm::setPolarity(Polarity polarity)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM polarity to %d", polarity);
}

void Pwm::enable()
{
  log_.log(hyped::core::LogLevel::kInfo, "Enabling PWM");
}

void Pwm::disable()
{
  log_.log(hyped::core::LogLevel::kInfo, "Disabling PWM");
}

}  // namespace hyped::io