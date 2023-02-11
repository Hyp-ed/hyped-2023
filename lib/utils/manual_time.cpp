
#include "manual_time.hpp"

#include <chrono>

#include <core/time.hpp>

namespace hyped::utils {

ManualTime::ManualTime() : current_time_{std::chrono::system_clock::from_time_t(0)}
{
}

core::TimePoint ManualTime::now() const
{
  return current_time_;
}

void ManualTime::setTime(const core::TimePoint time_point)
{
  current_time_ = time_point;
}

void ManualTime::setSecondsSinceEpoch(const std::uint64_t seconds_since_epoch)
{
  current_time_ = std::chrono::system_clock::time_point(std::chrono::seconds(seconds_since_epoch));
}

void ManualTime::addTime(const core::Duration duration)
{
  current_time_ += duration;
}

void ManualTime::addSeconds(const std::uint64_t num_seconds)
{
  current_time_ += core::durationFromSeconds(1);
}

}  // namespace hyped::utils
