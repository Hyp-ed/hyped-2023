#pragma once

#include "time.hpp"

#include <functional>

namespace hyped::core {

class Timer {
 public:
  Timer(const ITimeSource &time_source);
  Duration elapsed() const;

 private:
  const ITimeSource &time_source_;
  const TimePoint time_started_;
};

}  // namespace hyped::core
