#include "imu_displacement.hpp"

#include <cmath>

#include <chrono>

namespace hyped::navigation {

AccelerometerTrajectoryEstimator::AccelerometerTrajectoryEstimator(
  const core::ITimeSource &time, const core::TimePoint initial_time)
    : time_(time),
      previous_timestamp_(initial_time),
      imu_displacement_(0),
      imu_velocity_(0)
{
}

// TODOLater: change arguments to instead take some sort of "datapoint" struct with acc_val and
// timestamp instead
void AccelerometerTrajectoryEstimator::update(const core::Float imu_acceleration,
                                              const core::TimePoint imu_timestamp)
{
  core::Timer timer_(time_);
  const auto time_elapsed = timer_.measureElapsedTime(imu_timestamp, previous_timestamp_);
  const core::Float time_elapsed_seconds = timer_.elapsedTimeInSeconds(time_elapsed);

  // from equation v=u+at
  const core::Float velocity_estimate = imu_velocity_ + (imu_acceleration * time_elapsed_seconds);

  // from equation s = ut + 0.5*a*(t^2)
  imu_displacement_ = (imu_velocity_ * time_elapsed_seconds)
                      + (0.5 * imu_acceleration * time_elapsed_seconds * time_elapsed_seconds);

  imu_velocity_       = velocity_estimate;
  previous_timestamp_ = imu_timestamp;
}

core::Float AccelerometerTrajectoryEstimator::getAccelerometerDisplacement() const
{
  return imu_displacement_;
}

core::Float AccelerometerTrajectoryEstimator::getAccelerometerVelocity() const
{
  return imu_velocity_;
}
}  // namespace hyped::navigation