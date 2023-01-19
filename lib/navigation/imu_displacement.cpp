#include "imu_displacement.hpp"

#include <cmath>

#include <chrono>

namespace hyped::navigation {

ImuDisplacement::ImuDisplacement(const core::ITimeSource &time)
    : time_(time),
      imu_displacement_(0),
      imu_velocity_(0)
{
}

void ImuDisplacement::updateImuDisplacement(const core::Float imu_acceleration)
{
  const core::TimePoint time_now = time_.now();
  core::Timer timer_(time_);
  const core::Duration time_elapsed = timer_.measure_lapsed_time(previous_timestamp_);
  const std::uint64_t time_elapsed_seconds
    = std::chrono::duration_cast<std::chrono::seconds>(time_elapsed).count();

  // from equation v=u+at
  const core::Float velocity_estimate = imu_velocity_ + (imu_acceleration * time_elapsed_seconds);

  // from equation s = ut + 0.5*a*(t^2)
  imu_displacement_ = (imu_velocity_ * time_elapsed_seconds)
                      + (0.5 * imu_acceleration * time_elapsed_seconds * time_elapsed_seconds);

  // update class members
  imu_velocity_       = velocity_estimate;
  previous_timestamp_ = time_now;
}

void ImuDisplacement::initialiseTimePoint(const core::TimePoint initial_timepoint)
{
  // TODOLater: Implement this initialisation in main algorithm (some sort of startup process?,
  // Potentailly in initialisation state depending on how dependent navigation wants to be on state)
  previous_timestamp_ = initial_timepoint;
}

core::Float ImuDisplacement::getImuDisplacement()
{
  // TODOLater (/elsewhere?): error handling
  return imu_displacement_;
}

core::Float ImuDisplacement::getImuVelocity()
{
  // TODOLater (/elsewhere?): error handling
  return imu_velocity_;
}

}  // namespace hyped::navigation