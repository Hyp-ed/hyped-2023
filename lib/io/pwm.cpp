#include <fcntl.h>
#include <unistd.h>

#include "pwm.hpp"

namespace hyped::io {

Pwm::Pwm(const uint8_t channel, hyped::core::ILogger &log) : log_(log)
{
  log_.log(hyped::core::LogLevel::kInfo, "Pwm initiated on channel %d", channel);
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
  log_.log(hyped::core::LogLevel::kInfo, "Enabling PWM");
  return hyped::core::Result::kSuccess;
}

hyped::core::Result Pwm::disable()
{
  log_.log(hyped::core::LogLevel::kInfo, "Disabling PWM");
  return hyped::core::Result::kSuccess;
}

hyped::core::Result Pwm::setFrequency(uint32_t frequency)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting frequency to %d Hz", frequency);
  return hyped::core::Result::kSuccess;
}

std::optional<uint32_t> Pwm::getFrequency()
{
  log_.log(hyped::core::LogLevel::kInfo, "Getting frequency");
  return std::nullopt;
}

hyped::core::Result Pwm::setDutyCycle(float duty_cycle)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting duty cycle to %f", duty_cycle);
  return hyped::core::Result::kSuccess;
}

std::optional<float> Pwm::getDutyCycle()
{
  log_.log(hyped::core::LogLevel::kInfo, "Getting duty cycle");
  return std::nullopt;
}

hyped::core::Result Pwm::setPolarity(Polarity polarity)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM polarity to %d", polarity);
  return hyped::core::Result::kSuccess;
}

std::optional<Polarity> Pwm::getPolarity()
{
  log_.log(hyped::core::LogLevel::kInfo, "Getting PWM polarity");
  return std::nullopt;
}

hyped::core::Result Pwm::setPeriod(uint32_t period)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting period to %d", period);
  return hyped::core::Result::kSuccess;
}

std::optional<uint32_t> Pwm::getPeriod()
{
  log_.log(hyped::core::LogLevel::kInfo, "Getting period");
  return std::nullopt;
}

hyped::core::Result Pwm::run()
{
  log_.log(hyped::core::LogLevel::kInfo, "Running PWM");
  return hyped::core::Result::kSuccess;
}

}  // namespace hyped::io