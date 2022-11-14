#pragma once

#include "benchmark.hpp"

#include <core/time.hpp>
#include <navigation/navigator.hpp>
#include <utils/manual_time.hpp>

namespace hyped::navigation::benchmark {

class Benchmarker {
 public:
  Benchmarker(const core::ITimeSource &time_source_);

 private:
  const core::ITimeSource &time_source_;
  utils::ManualTime maunal_time_;
};

}  // namespace hyped::navigation::benchmark
