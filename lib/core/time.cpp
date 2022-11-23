#include "time.hpp"

namespace hyped::core {

TimePoint timePointFromSecondsSinceEpoch(const std::uint64_t num_seconds_since_epoch)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(num_seconds_since_epoch));
}

Duration durationFromSeconds(const std::uint64_t num_seconds)
{
  return num_seconds * kOneSecond;
}

}  // namespace hyped::core
