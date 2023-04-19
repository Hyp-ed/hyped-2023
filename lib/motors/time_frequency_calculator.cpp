#include "time_frequency_calculator.hpp"

namespace hyped::motors {

TimeFrequencyCalculator::TimeFrequencyCalculator(core::ILogger &logger) : logger_(logger)
{
  start_time_ = std::chrono::system_clock::now();
}

std::uint32_t TimeFrequencyCalculator::calculateFrequency(core::Float velocity)
{
  const std::uint32_t seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                          std::chrono::system_clock::now() - start_time_)
                                          .count();
  return seconds_elapsed * 2;
}

void TimeFrequencyCalculator::reset()
{
  start_time_ = std::chrono::system_clock::now();
}
}  // namespace hyped::motors
