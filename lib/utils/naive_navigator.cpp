#include "naive_navigator.hpp"

#include <cmath>

#include <numeric>

#include "core/time.hpp"

namespace hyped::utils {

NaiveNavigator::NaiveNavigator()
    : current_trajectory_{0, 0, 0},
      last_encoder_update_(core::timePointFromNanosSinceEpoch(0))
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
  core::Float sum = std::accumulate(encoder_data.value.begin(), encoder_data.value.end(), 0.0);
  core::Float encoder_average = static_cast<core::Float>(sum / core::kNumEncoders);
  // we assume that one revolution equals one metre
  current_trajectory_.displacement = encoder_average;
  current_trajectory_.velocity
    = encoder_average / ((last_encoder_update_ - encoder_data.measured_at) / core::kOneSecond);
}

void NaiveNavigator::accelerometerUpdate(
  const core::CombinedRawAccelerometerData &acceleration_data)
{
  core::Float sum = 0.0;
  for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
    const auto raw_acceleration = acceleration_data.value.at(i);
    std::uint64_t magnitude     = 0;
    magnitude += raw_acceleration.x * raw_acceleration.x;
    magnitude += raw_acceleration.y * raw_acceleration.y;
    magnitude += raw_acceleration.z * raw_acceleration.z;
    sum += std::sqrt(magnitude);
  }
  core::Float accelerometer_average = sum / static_cast<core::Float>(core::kNumAccelerometers);
  current_trajectory_.acceleration  = accelerometer_average;
  // TODOLater: improve this...
  current_trajectory_.velocity = 0;
}

}  // namespace hyped::utils
