#include "naive_navigator.hpp"

#include <cmath>

#include <numeric>

namespace hyped::utils {

NaiveNavigator::NaiveNavigator(const core::ITimeSource &time_source)
    : time_source_(time_source),
      current_trajectory_{0, 0, 0},
      encoder_update_timer_(time_source)
{
}

std::optional<core::Trajectory> NaiveNavigator::currentTrajectory()
{
  return current_trajectory_;
}

void NaiveNavigator::keyenceUpdate(const core::RawKeyenceData &keyence_data)
{
  // Do nothing. Keyence has no direct influence on trajectory
}

void NaiveNavigator::encoderUpdate(const core::RawEncoderData &encoder_data)
{
  const auto time_since_last_update = encoder_update_timer_.reset();
  core::Float sum             = std::accumulate(encoder_data.begin(), encoder_data.end(), 0.0);
  core::Float encoder_average = static_cast<core::Float>(sum / core::kNumEncoders);
  // we assume that one revolution equals one metre
  current_trajectory_.displacement = encoder_average;
  current_trajectory_.velocity     = encoder_average / (time_since_last_update / core::kSecond);
}

void NaiveNavigator::imuUpdate(const core::RawImuData &imu_data)
{
  core::Float sum = 0.0;
  for (std::size_t i = 0; i < core::kNumImus; ++i) {
    core::Float amplitude = 0.0;
    for (std::size_t j = 0; j < core::kNumAxis; ++j) {
      amplitude += imu_data.at(i).at(j) * imu_data.at(i).at(j);
    }
    sum += std::sqrt(amplitude);
  }
  core::Float imu_average          = sum / static_cast<core::Float>(core::kNumImus);
  current_trajectory_.acceleration = imu_average;
}

}  // namespace hyped::utils
