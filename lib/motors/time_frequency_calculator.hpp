#pragma once

#include "frequency_calculator.hpp"

#include <chrono>
#include <cstdint>

#include <core/logger.hpp>
#include <core/timer.hpp>

namespace hyped::motors {

class TimeFrequencyCalculator : public IFrequencyCalculator {
 public:
  TimeFrequencyCalculator(core::ILogger &logger);
  /**
   * @brief Returns the number of seconds elapsed as frequency mod 100
   *
   * @return core::Float equal to the passed in frequency
   */
  std::uint16_t calculateFrequency();

  /**
   * @brief Resets the start time to the current time
   */
  void reset();

 private:
  core::ILogger &logger_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
};

}  // namespace hyped::motors
