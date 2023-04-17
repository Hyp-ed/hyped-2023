#include "time_frequency_calculator.hpp"

namespace hyped::motors {

TimeFrequencyCalculator::TimeFrequencyCalculator(core::ILogger &logger) : logger_(logger)
{
  start_time_ = std::chrono::system_clock::now();
}

std::uint16_t TimeFrequencyCalculator::calculateFrequency(core::Float velocity)
{
  const std::uint64_t nanoseconds_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::system_clock::now() - start_time_)
                                              .count();
  return static_cast<core::Float>(nanoseconds_elapsed);
}

void TimeFrequencyCalculator::reset()
{
  start_time_ = std::chrono::system_clock::now();
}
}  // namespace hyped::motors
