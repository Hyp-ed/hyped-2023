#include "timer.hpp"

namespace hyped::core {

Timer::Timer(const ITimeSource &time) : time_(time)
{
}

Duration Timer::measure_execution_time(const std::function<void(void)> task)
{
  const auto before = time_.now();
  task();
  const auto after = time_.now();
  return after - before;
}

Duration Timer::measure_lapsed_time(const TimePoint previous_timepoint)
{
  const TimePoint time_now = time_.now();
  const Duration duration  = previous_timepoint - time_now;
  return duration;
}

}  // namespace hyped::core
