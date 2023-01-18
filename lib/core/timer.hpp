#pragma once

#include "time.hpp"

#include <functional>

namespace hyped::core {

class Timer {
 public:
  Timer(const ITimeSource &time);
  /**
   * @brief measure execution time of a function
   *
   * @param task function to measure execution time of
   * @return Duration time taken to execute function
   */
  Duration measure_execution_time(const std::function<void(void)> task);
  /**
   * @brief measure the duration between a previous timepoint and now
   *
   * @param previous_timepoint previous time stamp
   * @return Duration time between previous time point and now
   */
  Duration measure_lapsed_time(const TimePoint previous_timepoint);

 private:
  const ITimeSource &time_;
};

}  // namespace hyped::core
