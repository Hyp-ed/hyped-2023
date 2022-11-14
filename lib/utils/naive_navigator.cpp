#include "naive_navigator.hpp"

#include <numeric>

namespace hyped::utils {

NaiveNavigator::NaiveNavigator() : current_trajectory_{}
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
  core::Float sum                  = std::accumulate(encoder_data.begin(), encoder_data.end(), 0.0);
  core::Float encoder_average      = static_cast<core::Float>(sum / core::kNumEncoders);
  current_trajectory_.displacement = encoder_average;
}

void NaiveNavigator::imuUpdate(const core::RawImuData &imu_data)
{
  core::Float sum = 0.0;
  for (std::size_t i = 0; i < core::kNumImus; ++i) {
    for (std::size_t j = 0; j < core::kNumAxis; ++j) {
      sum += imu_data.at(i).at(j);
    }
  }
  core::Float imu_average          = static_cast<core::Float>(sum / core::kNumImus);
  current_trajectory_.acceleration = imu_average;
  // TODOLater: improve this...
  current_trajectory_.velocity = 0;
}

}  // namespace hyped::utils
