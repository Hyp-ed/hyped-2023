#pragma once

#include "data.hpp"

#include <map>
#include <vector>

#include <core/time.hpp>
#include <navigation/navigator.hpp>
#include <utils/manual_time.hpp>

namespace hyped::navigation::benchmark {

struct TrajectoryError {
  const core::Float displacement;
  const core::Float velocity;
  const core::Float acceleration;

  TrajectoryError(const core::Trajectory &expected, const core::Trajectory &actual);
};

struct Result {
  core::Duration total_time_taken;
  std::vector<core::Duration> time_taken_for_encoder_data;
  std::vector<core::Duration> time_taken_for_acceleration_data;
  std::vector<core::Duration> time_taken_for_keyence_data;
  std::vector<core::Duration> time_taken_for_trajectory;
  std::vector<std::optional<TrajectoryError>> trajectory_errors;
};

class Benchmark {
 public:
  // TODOLater: Use std::input_iterator and concepts to do this to avoid having to construct
  // these ugly maps.
  Benchmark(const core::ITimeSource &time_source, const Data &data);

  std::optional<Result> run(utils::ManualTime &manual_time, INavigator &navigator);

 private:
  const core::ITimeSource &time_source_;
  const Data data_;
  const std::vector<core::TimePoint> relevant_times_;
};

}  // namespace hyped::navigation::benchmark
