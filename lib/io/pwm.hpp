#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

enum class PwmModule {
  kECapPwm0 = 0,
  kECapPwm2,
  kEHRPwm0A,
  kEHRPwm0B,
  kEHRPwm1A,
  kEHRPwm1B,
  kEHRPwm2A,
  kEHRPwm2B
};
enum class Polarity { kActiveHigh = 0, kActiveLow };
enum class Mode { kStop = 0, kRun };
// use this class if a high‐frequency periodic switching signal is required
// PWM can achieve frequencies of 1 MHz or higher, without a significant CPU load
class Pwm {
 public:
  /**
   * @brief Create a PWM object and get all relevant file descriptors to do I/O operations
   * @param logger the logger to use
   * @param pwm_module the PWM module to use
   * @return a std::optional containing the PWM object if it was created successfully
   */
  static std::optional<Pwm> create(core::ILogger &logger, const PwmModule pwm_module);
  ~Pwm();

  /**
   * @brief Set the duty cycle of the PWM signal using a percentage
   * @param duty_cycle the duty cycle of the PWM signal
   * The valid values for duty cycle are 0.0 to 1.0 (0% to 100%)
   * @return kSuccess if the duty cycle was set successfully
   */
  core::Result setDutyCycleByPercentage(const core::Float duty_cycle);

  /**
   * @brief Set the duty cycle of the PWM signal using a value for active time
   * @param time_active the length of time the PWM signal is "active" in ns
   * The valid values for time active are 0 to the period (0% to 100%)
   * @return kSuccess if the duty cycle was set successfully
   */
  core::Result setDutyCycleByTime(const std::uint32_t time_active);

  /**
   * @brief Set the period of the PWM signal
   * @param period the period of the PWM signal in ns
   * @return kSuccess if the period was set successfully
   */
  core::Result setPeriod(const std::uint32_t period);

  /**
   * @brief Set the polarity of the PWM signal
   * @param polarity the polarity of the PWM signal
   * Polarity is either active high or active low
   * @return kSuccess if the polarity was set successfully
   */
  core::Result setPolarity(const Polarity polarity);

  /**
   * @brief Set the mode of the PWM signal
   * @param mode the mode of the PWM signal
   * Mode is either stop or run
   * @return kSuccess if the mode was set successfully
   */
  core::Result setMode(const Mode mode);

 private:
  Pwm(core::ILogger &logger,
      const int period_file,
      const int duty_cycle_file,
      const int polarity_file,
      const int enable_file);

  /**
   * @brief Get the corect folder name for the chosen PWM module
   * @param pwm_module the PWM module to get the folder name for
   * @return the folder name
   */
  static std::string getPwmFolderName(const PwmModule pwm_module);

  core::ILogger &logger_;
  std::uint32_t current_time_active_;  // ns
  std::uint32_t current_period_;       // ns
  Mode current_mode_;
  Polarity current_polarity_;
  int period_file_;
  int duty_cycle_file_;
  int polarity_file_;
  int enable_file_;
};

}  // namespace hyped::io
