#include "time_frequency_calculator.hpp"

namespace hyped::motors {

TimeFrequencyCalculator::TimeFrequencyCalculator(core::ILogger &logger) : logger_(logger)
{
  start_time_ = std::chrono::system_clock::now();
}

std::uint16_t TimeFrequencyCalculator::calculateFrequency()
{
  return static_cast<std::uint16_t>((((std::chrono::system_clock::now() - start_time_) * 2) % 120)
                                    / 1'000'000'000);
}

void TimeFrequencyCalculator::reset()
{
  start_time_ = std::chrono::system_clock::now();
}
}  // namespace hyped::motors
