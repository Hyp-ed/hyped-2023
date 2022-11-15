#pragma once

#include <functional>
#include <map>
#include <memory>
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
  std::vector<core::Duration> time_taken_for_imu_data;
  std::vector<core::Duration> time_taken_for_keyence_data;
  std::vector<core::Duration> time_taken_for_trajectory;
  std::vector<std::optional<TrajectoryError>> trajectory_errors;
};

class Benchmark {
 public:
  // TODOLater: Use std::input_iterator and concepts to do this to avoid having to construct
  // these ugly maps.
  Benchmark(const core::ITimeSource &time_source,
            const std::map<core::TimePoint, core::RawEncoderData> &encoder_data_by_time,
            const std::map<core::TimePoint, core::RawImuData> &imu_data_by_time,
            const std::map<core::TimePoint, core::RawKeyenceData> &keyence_data_by_time,
            const std::map<core::TimePoint, core::Trajectory> &trajectory_data_by_time);

  std::optional<Result> run(utils::ManualTime &manual_time, INavigator &navigator);

 private:
  static std::vector<core::TimePoint> getRelevantTimes(
    const std::map<core::TimePoint, core::RawEncoderData> &encoder_data_by_time,
    const std::map<core::TimePoint, core::RawImuData> &imu_data_by_time,
    const std::map<core::TimePoint, core::RawKeyenceData> &keyence_data_by_time,
    const std::map<core::TimePoint, core::Trajectory> &trajectory_by_time);

  const core::ITimeSource &time_source_;
  const std::map<core::TimePoint, core::RawEncoderData> encoder_data_by_time_;
  const std::map<core::TimePoint, core::RawImuData> imu_data_by_time_;
  const std::map<core::TimePoint, core::RawKeyenceData> keyence_data_by_time_;
  const std::map<core::TimePoint, core::Trajectory> trajectory_by_time_;
  const std::vector<core::TimePoint> relevant_times_;
};

}  // namespace hyped::navigation::benchmark
