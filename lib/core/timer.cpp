#include "timer.hpp"

namespace hyped::core {

Timer::Timer(const ITimeSource &time_source)
    : time_source_(time_source),
      time_started_(time_source.now())
{
}

Duration Timer::elapsed() const
{
  return time_source_.now() - time_started_;
}

Duration Timer::reset()
{
  const auto previous_time_started = time_started_;
  time_started_                    = time_source_.now();
  return time_started_ - previous_time_started;
}

}  // namespace hyped::core
