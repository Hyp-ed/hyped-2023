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

PwmWriteResult Pwm::setFrequency(uint16_t frequency)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM frequency to %d", frequency);
}

PwmWriteResult Pwm::setDutyCycle(float duty_cycle)
{
  if (duty_cycle < 0 || duty_cycle > 1) {
    log_.log(hyped::core::LogLevel::kFatal, "Duty cycle must be between 0 and 1");
    return PwmWriteResult::kError;;
  }
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM duty cycle to %f", duty_cycle);
  const int num_bytes_written = write(dutyFd_, std::to_string(duty_cycle).c_str(), 4);
  if (num_bytes_written < 0) {
    log_.log(hyped::core::LogLevel::kFatal, "Failed to write to duty cycle file");
    return PwmWriteResult::kError;
  }
  return PwmWriteResult::kSuccess;
}

PwmWriteResult Pwm::setPolarity(Polarity polarity)
{
  log_.log(hyped::core::LogLevel::kInfo, "Setting PWM polarity to %d", polarity);
}

PwmWriteResult Pwm::enable()
{
  const int num_bytes_written = write(enableFd_, "1", 1);
  if (num_bytes_written < 0) {
    log_.log(hyped::core::LogLevel::kFatal, "Failed to write to enable file");
    return PwmWriteResult::kError;
  }
  return PwmWriteResult::kSuccess;
}

PwmWriteResult Pwm::disable()
{
  const int num_bytes_written = write(enableFd_, "0", 1);
  if (num_bytes_written < 0) {
    log_.log(hyped::core::LogLevel::kFatal, "Failed to write to enable file");
    return PwmWriteResult::kError;
  }
  return PwmWriteResult::kSuccess;
}

}  // namespace hyped::io