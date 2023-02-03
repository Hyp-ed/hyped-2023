#pragma once

#include "time.hpp"
#include "types.hpp"

#include <functional>

namespace hyped::core {

class Timer {
 public:
  Timer(const ITimeSource &time);

  Duration measureExecutionTime(const std::function<void(void)> task);

  Duration elapsed(const TimePoint current_timepoint, const TimePoint previous_timepoint);

  Float elapsedTimeInSeconds(const Duration time_elapsed);

 private:
  const ITimeSource &time_;
  static constexpr std::uint32_t kOneSecond = 1'000'000'000;
};

}  // namespace hyped::core
