#include "time.hpp"

namespace hyped::core {

TimePoint timePointFromSecondsSinceEpoch(const std::uint64_t num_seconds_since_epoch)
{
  return std::chrono::system_clock::time_point(std::chrono::seconds(num_seconds_since_epoch));
}

TimePoint timePointFromNanosSinceEpoch(const std::uint64_t num_seconds_since_epoch)
{
#ifdef __linux__
  return std::chrono::system_clock::time_point(std::chrono::nanoseconds(num_seconds_since_epoch));
#else
  return std::chrono::system_clock::time_point(
    std::chrono::microseconds(num_seconds_since_epoch / 1000));
#endif
}

Duration durationFromSeconds(const std::uint64_t num_seconds)
{
  return num_seconds * kOneSecond;
}

}  // namespace hyped::core
