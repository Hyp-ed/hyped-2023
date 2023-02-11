#pragma once

#include <chrono>
namespace hyped::core {

/**
 * @brief Absolute point in time; nanosecond precision.
 */
using TimePoint = std::chrono::system_clock::time_point;

TimePoint timePointFromSecondsSinceEpoch(const std::uint64_t num_seconds_since_epoch);

TimePoint timePointFromNanosSinceEpoch(const std::uint64_t num_seconds_since_epoch);

/**
 * @brief Difference between points in time; nanosecond precision.
 */
using Duration = std::chrono::system_clock::duration;

static constexpr Duration kOneSecond = static_cast<Duration>(1'000'000'000);

Duration durationFromSeconds(const std::uint64_t num_seconds);

/**
 * @brief Time provider allowing the user to obtain the current time of the
 * system. We abtract this away instead of using `WallClock` directly in order to
 * allow testability.
 */
class ITimeSource {
 public:
  virtual TimePoint now() const = 0;
};

}  // namespace hyped::core
