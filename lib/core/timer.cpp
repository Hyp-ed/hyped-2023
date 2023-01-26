#include "timer.hpp"

namespace hyped::core {

Timer::Timer(const ITimeSource &time) : time_(time)
{
}

Duration Timer::measureExecutionTime(const std::function<void(void)> task)
{
  const auto before = time_.now();
  task();
  const auto after = time_.now();
  return after - before;
}

Duration Timer::measureElapsedTime(const TimePoint current_timepoint,
                                   const TimePoint previous_timepoint)
{
  const Duration duration = previous_timepoint - previous_timepoint;
  return duration;
}

Float Timer::elapsedTimeInSeconds(const Duration time_elapsed)
{
  std::uint64_t nanoseconds_elapsed
    = std::chrono::duration_cast<std::chrono::nanoseconds>(time_elapsed).count();
  return static_cast<Float>(nanoseconds_elapsed) / kOneSecond;
}

}  // namespace hyped::core
