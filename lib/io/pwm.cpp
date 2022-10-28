#include <fcntl.h>
#include <unistd.h>

#include "pwm.hpp"

namespace hyped::io {

Pwm::Pwm(const uint8_t channel, hyped::core::ILogger &log) : log_(log)
{
  log_.log(hyped::core::LogLevel::kDebug, "Pwm initiated on channel %d", channel);
  path_ = "/sys/class/pwm/pwmchip" + std::to_string(channel) + "/pwm" + std::to_string(channel);
  enableFd_ = open((path_ + "/enable").c_str(), O_RDWR);
  dutyFd_ = open((path_ + "/duty_cycle").c_str(), O_RDWR);
  periodFd_ = open((path_ + "/period").c_str(), O_RDWR);
  polarityFd_ = open((path_ + "/polarity").c_str(), O_RDWR);
}

Pwm::~Pwm()
{
  close(enableFd_);
  close(dutyFd_);
  close(periodFd_);
  close(polarityFd_);
}

hyped::core::Result Pwm::enable()
{
  log_.log(hyped::core::LogLevel::kDebug, "Enabling PWM");
  return hyped::core::Result::kSuccess;
}

hyped::core::Result Pwm::disable()
{
  log_.log(hyped::core::LogLevel::kDebug, "Disabling PWM");
  return hyped::core::Result::kSuccess;
}

hyped::core::Result Pwm::setFrequency(uint32_t frequency)
{
  log_.log(hyped::core::LogLevel::kDebug, "Setting frequency to %d Hz", frequency);
  return hyped::core::Result::kSuccess;
}

std::optional<uint32_t> Pwm::getFrequency()
{
  return std::nullopt;
}

hyped::core::Result Pwm::setDutyCycle(float duty_cycle)
{
  log_.log(hyped::core::LogLevel::kDebug, "Setting duty cycle to %f", duty_cycle);
  return hyped::core::Result::kSuccess;
}

std::optional<float> Pwm::getDutyCycle()
{
  return std::nullopt;
}

hyped::core::Result Pwm::setPolarity(Polarity polarity)
{
  log_.log(hyped::core::LogLevel::kDebug, "Setting PWM polarity to %d", polarity);
  return hyped::core::Result::kSuccess;
}

std::optional<Polarity> Pwm::getPolarity()
{
  return std::nullopt;
}

hyped::core::Result Pwm::setPeriod(uint32_t period)
{
  log_.log(hyped::core::LogLevel::kDebug, "Setting period to %d", period);
  return hyped::core::Result::kSuccess;
}

std::optional<uint32_t> Pwm::getPeriod()
{
  return std::nullopt;
}

hyped::core::Result Pwm::run()
{
  return hyped::core::Result::kSuccess;
}

}  // namespace hyped::io