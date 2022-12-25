#include "pwm.hpp"

namespace hyped::io {

Pwm::Pwm(core::Logger &logger, const PwmOutput pwm_output) : logger_(logger), period_(0)
{
  std::string pwm_address = "/sys/class/pwm/" + getPwmFolderName(pwm_output) + "/";
  // First get the file descriptor for the period file
  std::string period_address = pwm_address + "period";
  period_file_               = open(period_address.c_str(), O_WRONLY);
  if (period_file_ < 0) { logger_.log(core::LogLevel::kFatal, "Unable to open PWM period file"); }
  // Then get the file descriptor for the duty cycle file
  std::string duty_cycle_address = pwm_address + "duty_cycle";
  duty_cycle_file_               = open(duty_cycle_address.c_str(), O_WRONLY);
  if (duty_cycle_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to open PWM duty cycle file");
  }
  // Next get the file descriptor for the polarity file
  std::string polarity_address = pwm_address + "polarity";
  polarity_file_               = open(polarity_address.c_str(), O_WRONLY);
  if (polarity_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to open PWM polarity file");
  }
  // Finally get the file descriptor for the enable file
  std::string enable_address = pwm_address + "enable";
  enable_file_               = open(enable_address.c_str(), O_WRONLY);
  if (enable_file_ < 0) { logger_.log(core::LogLevel::kFatal, "Unable to open PWM enable file"); }
}

Pwm::~Pwm()
{
  close(period_file_);
  close(duty_cycle_file_);
  close(polarity_file_);
  close(enable_file_);
}

core::Result Pwm::setDutyCycleByPercentage(const core::Float duty_cycle)
{
  if (duty_cycle > 1.0) {
    logger_.log(core::LogLevel::kFatal, "Duty cycle cannot be greater than 1.0");
    return core::Result::kFailure;
  }
  if (duty_cycle < 0.0) {
    logger_.log(core::LogLevel::kFatal, "Duty cycle cannot be less than 0.0");
    return core::Result::kFailure;
  }
  const auto time_active = static_cast<std::uint32_t>(duty_cycle * period_);
  return setDutyCycleByTime(time_active);
}

core::Result Pwm::setDutyCycleByTime(const std::uint32_t time_active)
{
  if (duty_cycle_file_ < 0) {
    logger_.log(core::LogLevel::kFatal,
                "Could not find PWM duty cycle file while setting duty cycle");
    return core::Result::kFailure;
  }
  if (time_active > period_) {
    logger_.log(core::LogLevel::kFatal, "Duty cycle cannot be greater than period");
    return core::Result::kFailure;
  }
  const auto num_bytes_written = write(duty_cycle_file_, &time_active, sizeof(time_active));
  if (num_bytes_written != sizeof(time_active)) {
    logger_.log(core::LogLevel::kFatal, "Could not write to PWM duty cycle file");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

core::Result Pwm::setPeriod(const std::uint32_t period)
{
  if (period_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Could not find PWM period file while setting period");
    return core::Result::kFailure;
  }
  period_                      = period;
  const auto num_bytes_written = write(period_file_, &period_, sizeof(period_));
  if (num_bytes_written != sizeof(period_)) {
    logger_.log(core::LogLevel::kFatal, "Could not write to PWM period file");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

core::Result Pwm::setPolarity(const Polarity polarity)
{
  if (polarity_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Could not find PWM polarity file while setting polarity");
    return core::Result::kFailure;
  }
  std::uint8_t polarity_value  = polarity == Polarity::kActiveLow ? 0 : 1;
  const auto num_bytes_written = write(polarity_file_, &polarity_value, sizeof(polarity_value));
  if (num_bytes_written != sizeof(polarity_value)) {
    logger_.log(core::LogLevel::kFatal, "Could not write to PWM polarity file");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

core::Result Pwm::setMode(const Mode mode)
{
  if (enable_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Could not find PWM enable file while setting run mode");
    return core::Result::kFailure;
  }
  std::uint8_t mode_value      = mode == Mode::kStop ? 0 : 1;
  const auto num_bytes_written = write(enable_file_, &mode_value, sizeof(mode_value));
  if (num_bytes_written != sizeof(mode_value)) {
    logger_.log(core::LogLevel::kFatal, "Could not write to PWM enable file");
    return core::Result::kFailure;
  }
  return core::Result::kSuccess;
}

std::string Pwm::getPwmFolderName(const PwmOutput pwm_output)
{
  switch (pwm_output) {
    case PwmOutput::kECapPwm0:
      return "pwm-0:0";
    case PwmOutput::kECapPwm2:
      // TODO: Check this particular case in TLD
      return "pwm-6:0";
    case PwmOutput::kEHRPwm0A:
      return "pwm-1:0";
    case PwmOutput::kEHRPwm0B:
      return "pwm-1:1";
    case PwmOutput::kEHRPwm1A:
      return "pwm-4:0";
    case PwmOutput::kEHRPwm1B:
      return "pwm-4:1";
    case PwmOutput::kEHRPwm2A:
      return "pwm-7:0";
    case PwmOutput::kEHRPwm2B:
      return "pwm-7:1";
    default:
      return "pwm-0:0";
  }
}

}  // namespace hyped::io