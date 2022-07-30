#pragma once

#include "time.hpp"

#include <functional>

namespace hyped::core {

class Timer {
 public:
  Timer(const Time &time);
  Duration measure_execution_time(const std::function<void(void)> task);

 private:
  const Time &time_;
};

}  // namespace hyped::core
