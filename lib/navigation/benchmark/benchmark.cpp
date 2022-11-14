#include "benchmark.hpp"

#include <core/timer.hpp>

namespace hyped::navigation::benchmark {

TrajectoryError::TrajectoryError(const core::Trajectory &expected, const core::Trajectory &actual)
    : displacement(std::abs(expected.displacement - actual.displacement)),
      velocity(std::abs(expected.velocity - actual.velocity)),
      acceleration(std::abs(expected.acceleration - actual.acceleration))
{
}

Benchmark::Benchmark(const core::ITimeSource &time_source,
                     const std::map<core::TimePoint, core::RawEncoderData> &encoder_data_by_time,
                     const std::map<core::TimePoint, core::RawImuData> &imu_data_by_time,
                     const std::map<core::TimePoint, core::RawKeyenceData> &keyence_data_by_time,
                     const std::map<core::TimePoint, core::Trajectory> &trajectory_by_time)
    : time_source_(time_source),
      encoder_data_by_time_(encoder_data_by_time),
      imu_data_by_time_(imu_data_by_time),
      keyence_data_by_time_(keyence_data_by_time),
      trajectory_by_time_(trajectory_by_time),
      relevant_times_(getRelevantTimes(
        encoder_data_by_time, imu_data_by_time, keyence_data_by_time, trajectory_by_time))
{
}

std::vector<core::TimePoint> Benchmark::getRelevantTimes(
  const std::map<core::TimePoint, core::RawEncoderData> &encoder_data_by_time,
  const std::map<core::TimePoint, core::RawImuData> &imu_data_by_time,
  const std::map<core::TimePoint, core::RawKeyenceData> &keyence_data_by_time,
  const std::map<core::TimePoint, core::Trajectory> &trajectory_by_time)
{
  std::vector<core::TimePoint> times(encoder_data_by_time.size() + imu_data_by_time.size()
                                     + keyence_data_by_time.size() + trajectory_by_time.size());
  for (const auto &[time_point, _] : encoder_data_by_time) {
    times.push_back(time_point);
  }
  for (const auto &[time_point, _] : imu_data_by_time) {
    times.push_back(time_point);
  }
  for (const auto &[time_point, _] : keyence_data_by_time) {
    times.push_back(time_point);
  }
  for (const auto &[time_point, _] : trajectory_by_time) {
    times.push_back(time_point);
  }
  std::sort(times.begin(), times.end());
  return times;
}

// TODOLater: Allow this to operate using constructors rather than already initialised navigators.
// This would have two benefits:
// 1. The constraints and relations of the input arguments would be more clear.
// 2. We could measure and compare the initialisation time taken by each of the navigators.
std::optional<Result> Benchmark::run(utils::ManualTime &manual_time, INavigator &navigator)
{
  if (manual_time.now() != std::chrono::system_clock::from_time_t(0)) { return std::nullopt; }
  const core::Timer full_benchmark_timer(time_source_);
  Result result;
  for (const auto &time_point : relevant_times_) {
    if (encoder_data_by_time_.contains(time_point)) {
      const auto encoder_data = encoder_data_by_time_.find(time_point)->second;
      const core::Timer encoder_data_timer(time_source_);
      navigator.encoderUpdate(encoder_data);
      result.time_taken_for_encoder_data.push_back(encoder_data_timer.elapsed());
    }
    if (imu_data_by_time_.contains(time_point)) {
      const auto imu_data = imu_data_by_time_.find(time_point)->second;
      const core::Timer imu_data_timer(time_source_);
      navigator.imuUpdate(imu_data);
      result.time_taken_for_imu_data.push_back(imu_data_timer.elapsed());
    }
    if (keyence_data_by_time_.contains(time_point)) {
      const auto keyence_data = keyence_data_by_time_.find(time_point)->second;
      const core::Timer keyence_data_timer(time_source_);
      navigator.keyenceUpdate(keyence_data);
      result.time_taken_for_keyence_data.push_back(keyence_data_timer.elapsed());
    }
    if (trajectory_by_time_.contains(time_point)) {
      const auto expected_trajectory = trajectory_by_time_.find(time_point)->second;
      const core::Timer trajectory_timer(time_source_);
      const auto calculated_trajectory = navigator.currentTrajectory();
      if (calculated_trajectory) {
        result.trajectory_errors.emplace_back(
          TrajectoryError(expected_trajectory, *calculated_trajectory));
      } else {
        result.trajectory_errors.push_back(std::nullopt);
      }
      result.time_taken_for_trajectory.push_back(trajectory_timer.elapsed());
    }
  }
  result.total_time_taken = full_benchmark_timer.elapsed();
  return result;
}

}  // namespace hyped::navigation::benchmark
