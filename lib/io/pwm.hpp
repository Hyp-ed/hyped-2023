#pragma once

#include <fcntl.h>   // define O_WONLY and O_RDONLY
#include <unistd.h>  // close()

#include <cstdint>
#include <string>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

enum class PwmOutput {
  kECapPwm0 = 0,
  kECapPwm2,
  kEHRPwm0A,
  kEHRPwm0B,
  kEHRPwm1A,
  kEHRPwm1B,
  kEHRPwm2A,
  kEHRPwm2B
};
enum class Polarity { kActiveLow = 0, kActiveHigh };
enum class Mode { kStop = 0, kRun };
// use this class if a high‐frequency periodic switching signal is required
// PWM can achieve frequencies of 1 MHz or higher, without a significant CPU load
class Pwm {
 public:
  Pwm(core::Logger &logger, const PwmOutput pwm_output);
  ~Pwm();

  // The valid values for duty cycle are 0.0 to 1.0 (0% to 100%)
  core::Result setDutyCycleByPercentage(const core::Float duty_cycle);
  // The valid values for time active are 0 to the period (0% to 100%)
  core::Result setDutyCycleByTime(const std::uint32_t time_active);
  core::Result setPeriod(const std::uint32_t period);
  core::Result setPolarity(const Polarity polarity);
  core::Result setMode(const Mode mode);
  std::string getPwmFolderName(const PwmOutput pwm_output);

 private:
  core::Logger &logger_;
  std::uint32_t period_;  // ns
  int period_file_;
  int duty_cycle_file_;
  int polarity_file_;
  int enable_file_;
};

}  // namespace hyped::io
