#include "imu_displacement.hpp"

#include <cmath>

#include <chrono>

namespace hyped::navigation {

ImuDisplacement::ImuDisplacement(const core::ITimeSource &time, const core::TimePoint initial_time)
    : time_(time),
      previous_timestamp_(initial_time),
      imu_displacement_(0),
      imu_velocity_(0)
{
}

void ImuDisplacement::updateImuDisplacement(const core::Float imu_acceleration,
                                            const core::TimePoint imu_timestamp)
{
  core::Timer timer_(time_);
  const core::Duration time_elapsed = timer_.measureElapsedTime(imu_timestamp, previous_timestamp_);
  const core::Float time_elapsed_seconds = timer_.elapsedTimeInSeconds(time_elapsed);

  // from equation v=u+at
  const core::Float velocity_estimate = imu_velocity_ + (imu_acceleration * time_elapsed_seconds);

  // from equation s = ut + 0.5*a*(t^2)
  imu_displacement_ = (imu_velocity_ * time_elapsed_seconds)
                      + (0.5 * imu_acceleration * time_elapsed_seconds * time_elapsed_seconds);

  imu_velocity_       = velocity_estimate;
  previous_timestamp_ = imu_timestamp;
}

core::Float ImuDisplacement::getImuDisplacement()
{
  return imu_displacement_;
}

core::Float ImuDisplacement::getImuVelocity()
{
  return imu_velocity_;
}
}  // namespace hyped::navigation