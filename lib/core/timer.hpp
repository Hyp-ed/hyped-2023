#pragma once

#include "time.hpp"

#include <functional>

namespace hyped::core {

class Timer {
 public:
  Timer(const ITimeSource &time);
  Duration measure_execution_time(const std::function<void(void)> task);

 private:
  const ITimeSource &time_;
};

}  // namespace hyped::core
