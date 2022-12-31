#include "pwm.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

std::optional<Pwm> Pwm::create(core::Logger &logger, const PwmModule pwm_module)
{
  Pwm pwm(logger, pwm_module);
  const auto initialisation_result = pwm.initialise();
  if (initialisation_result == core::Result::kFailure) {
    pwm.logger_.log(core::LogLevel::kFatal, "Failed to initialise PWM");
    return std::nullopt;
  }
  pwm.logger_.log(core::LogLevel::kDebug, "Successfully initialised PWM");
  return pwm;
}

Pwm::Pwm(core::Logger &logger, const PwmModule pwm_module)
    : logger_(logger),
      pwm_module_(pwm_module),
      current_time_active_(0),
      current_period_(0),
      current_mode_(Mode::kStop),
      current_polarity_(Polarity::kActiveHigh)
{
}

Pwm::~Pwm()
{
  close(period_file_);
  close(duty_cycle_file_);
  close(polarity_file_);
  close(enable_file_);
}

core::Result Pwm::initialise()
{
  const std::string pwm_address = "/sys/class/pwm/" + getPwmFolderName(pwm_module_) + "/";
  // First get the file descriptor for the period file
  const std::string period_address = pwm_address + "period";
  period_file_                     = open(period_address.c_str(), O_WRONLY);
  if (period_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open PWM period file");
    return core::Result::kFailure;
  }
  // Then get the file descriptor for the duty cycle file
  const std::string duty_cycle_address = pwm_address + "duty_cycle";
  duty_cycle_file_                     = open(duty_cycle_address.c_str(), O_WRONLY);
  if (duty_cycle_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open PWM duty cycle file");
    return core::Result::kFailure;
  }
  // Next get the file descriptor for the polarity file
  const std::string polarity_address = pwm_address + "polarity";
  polarity_file_                     = open(polarity_address.c_str(), O_WRONLY);
  if (polarity_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open PWM polarity file");
    return core::Result::kFailure;
  }
  // Finally get the file descriptor for the enable file
  const std::string enable_address = pwm_address + "enable";
  enable_file_                     = open(enable_address.c_str(), O_WRONLY);
  if (enable_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open PWM enable file");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully initialised PWM");
  return core::Result::kSuccess;
}

core::Result Pwm::setDutyCycleByPercentage(const core::Float duty_cycle)
{
  // Ensure duty cycle is between 0 and 1 inlcusive
  if (duty_cycle > 1.0) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to set duty cycle, percentage cannot be greater than 1.0");
    return core::Result::kFailure;
  }
  if (duty_cycle < 0.0) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to set duty cycle, percentage cannot be less than 0.0");
    return core::Result::kFailure;
  }
  const auto time_active = static_cast<std::uint32_t>(duty_cycle * current_period_);
  return setDutyCycleByTime(time_active);
}

core::Result Pwm::setDutyCycleByTime(const std::uint32_t time_active)
{
  if (duty_cycle_file_ < 0) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to find PWM duty cycle file while setting duty cycle");
    return core::Result::kFailure;
  }
  // Ensure active time is less or equal to period
  if (time_active > current_period_) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to set duty cycle, active time cannot be greater than period");
    return core::Result::kFailure;
  }
  // Avoid I/O if the duty cycle is the same
  if (time_active == current_time_active_) {
    logger_.log(core::LogLevel::kDebug, "Duty cycle is the same as before, skipping I/O");
    return core::Result::kSuccess;
  }
  current_time_active_ = time_active;
  const auto num_bytes_written
    = write(duty_cycle_file_, &current_time_active_, sizeof(current_time_active_));
  if (num_bytes_written != sizeof(current_time_active_)) {
    logger_.log(core::LogLevel::kFatal, "Failed to write to PWM duty cycle file");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully set PWM active time to %d", time_active);
  return core::Result::kSuccess;
}

core::Result Pwm::setPeriod(const std::uint32_t period)
{
  if (period_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to find PWM period file while setting period");
    return core::Result::kFailure;
  }
  // Avoid I/O if no change is required
  if (current_period_ == period) {
    logger_.log(core::LogLevel::kDebug, "PWM period is already set to %d", period);
    return core::Result::kSuccess;
  }
  current_period_              = period;
  const auto num_bytes_written = write(period_file_, &current_period_, sizeof(current_period_));
  if (num_bytes_written != sizeof(current_period_)) {
    logger_.log(core::LogLevel::kFatal, "Failed to write to PWM period file");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully set PWM period to %d", current_period_);
  return core::Result::kSuccess;
}

core::Result Pwm::setPolarity(const Polarity polarity)
{
  if (polarity_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to find PWM polarity file while setting polarity");
    return core::Result::kFailure;
  }
  // Avoid I/O if no change is required
  if (current_polarity_ == polarity) {
    logger_.log(core::LogLevel::kDebug, "PWM polarity is already set to %d", polarity);
    return core::Result::kSuccess;
  }
  current_polarity_                 = polarity;
  const std::uint8_t polarity_value = static_cast<std::uint8_t>(current_polarity_);
  const auto num_bytes_written = write(polarity_file_, &polarity_value, sizeof(polarity_value));
  if (num_bytes_written != sizeof(polarity_value)) {
    logger_.log(core::LogLevel::kFatal, "Failed to write to PWM polarity file");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully set PWM polarity to %d", polarity_value);
  return core::Result::kSuccess;
}

core::Result Pwm::setMode(const Mode mode)
{
  if (enable_file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to find PWM enable file while setting mode");
    return core::Result::kFailure;
  }
  // Avoid I/O if no change is required
  if (current_mode_ == mode) {
    logger_.log(core::LogLevel::kDebug, "PWM mode is already set to %d", mode);
    return core::Result::kSuccess;
  }
  current_mode_                 = mode;
  const std::uint8_t mode_value = static_cast<std::uint8_t>(mode);
  const auto num_bytes_written  = write(enable_file_, &mode_value, sizeof(mode_value));
  if (num_bytes_written != sizeof(mode_value)) {
    logger_.log(core::LogLevel::kFatal, "Failed to write to PWM enable file");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully set PWM mode to %d", mode_value);
  return core::Result::kSuccess;
}

std::string Pwm::getPwmFolderName(const PwmModule pwm_module)
{
  switch (pwm_module) {
    case PwmModule::kECapPwm0:
      return "pwm-0:0";
    case PwmModule::kECapPwm2:
      // TODO: Check this particular case in TLD
      return "pwm-6:0";
    case PwmModule::kEHRPwm0A:
      return "pwm-1:0";
    case PwmModule::kEHRPwm0B:
      return "pwm-1:1";
    case PwmModule::kEHRPwm1A:
      return "pwm-4:0";
    case PwmModule::kEHRPwm1B:
      return "pwm-4:1";
    case PwmModule::kEHRPwm2A:
      return "pwm-7:0";
    case PwmModule::kEHRPwm2B:
      return "pwm-7:1";
  }
}

}  // namespace hyped::io