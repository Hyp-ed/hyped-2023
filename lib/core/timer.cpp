#include "timer.hpp"

namespace hyped::core {

Timer::Timer(const ITimeSource &time_source) : time_source_(time_source), time_started_(time_source.now())
{
}

Duration Timer::elapsed() const
{
  return time_source_.now() - time_started_;
}

}  // namespace hyped::core
