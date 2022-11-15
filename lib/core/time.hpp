#pragma once

#include <chrono>
namespace hyped::core {

/**
 * @brief Absolute point in time; nanosecond precision.
 */
using TimePoint = std::chrono::system_clock::time_point;

/**
 * @brief Difference between points in time; nanosecond precision.
 */
using Duration = std::chrono::system_clock::duration;

static constexpr Duration kSecond = static_cast<Duration>(1000000000);

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
