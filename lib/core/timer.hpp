#pragma once

#include "time.hpp"

#include <functional>

namespace hyped::core {

class Timer {
 public:
  Timer(const ITimeSource &time_source);
  Duration elapsed() const;
  Duration reset();

 private:
  const ITimeSource &time_source_;
  TimePoint time_started_;
};

}  // namespace hyped::core
