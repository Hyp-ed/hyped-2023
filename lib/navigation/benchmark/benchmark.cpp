#include "benchmark.hpp"

#include <core/timer.hpp>

namespace hyped::navigation::benchmark {

TrajectoryError::TrajectoryError(const core::Trajectory &expected, const core::Trajectory &actual)
    : displacement(std::abs(expected.displacement - actual.displacement)),
      velocity(std::abs(expected.velocity - actual.velocity)),
      acceleration(std::abs(expected.acceleration - actual.acceleration))
{
}

Benchmark::Benchmark(const core::ITimeSource &time_source, const Data &data)
    : time_source_(time_source),
      data_(data),
      relevant_times_(data.getRelevantTimes())
{
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
    const auto encoder_data = data_.getEncoderDataAt(time_point);
    if (encoder_data) {
      const core::Timer encoder_data_timer(time_source_);
      navigator.encoderUpdate(*encoder_data);
      result.time_taken_for_encoder_data.push_back(encoder_data_timer.elapsed());
    }
    const auto acceleration_data = data_.getAccelerationDataAt(time_point);
    if (acceleration_data) {
      const core::Timer acceleration_data_timer(time_source_);
      navigator.accelerometerUpdate(*acceleration_data);
      result.time_taken_for_acceleration_data.push_back(acceleration_data_timer.elapsed());
    }
    const auto keyence_data = data_.getKeyenceDataAt(time_point);
    if (keyence_data) {
      const core::Timer keyence_data_timer(time_source_);
      navigator.keyenceUpdate(*keyence_data);
      result.time_taken_for_keyence_data.push_back(keyence_data_timer.elapsed());
    }
    const auto expected_trajectory = data_.getTrajectoryDataAt(time_point);
    if (expected_trajectory) {
    const core::Timer trajectory_timer(time_source_);
    const auto calculated_trajectory = navigator.currentTrajectory();
    if (calculated_trajectory) {
      result.trajectory_errors.emplace_back(
        TrajectoryError(*expected_trajectory, *calculated_trajectory));
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
