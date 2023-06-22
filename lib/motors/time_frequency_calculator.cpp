#include "time_frequency_calculator.hpp"

namespace hyped::motors {

TimeFrequencyCalculator::TimeFrequencyCalculator(core::ILogger &logger) : logger_(logger)
{
  start_time_ = std::chrono::system_clock::now();
}

std::uint32_t TimeFrequencyCalculator::calculateFrequency(core::Float velocity)
{
  const std::uint32_t milliseconds_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                               std::chrono::system_clock::now() - start_time_)
                                               .count();
  if (milliseconds_elapsed < 1000) { return milliseconds_elapsed; }
  if ((milliseconds_elapsed > 1000) && (milliseconds_elapsed < 2000)) { return 1000; }
  if ((milliseconds_elapsed > 2000) && (milliseconds_elapsed < 3000)) {
    return 1000 - (milliseconds_elapsed % 1000);
  }
  return 0;
}

void TimeFrequencyCalculator::reset()
{
  start_time_ = std::chrono::system_clock::now();
}
}  // namespace hyped::motors
