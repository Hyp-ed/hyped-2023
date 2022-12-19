#include "imu_displacement.hpp"

#include <cmath>

#include <chrono>

namespace hyped::navigation {

ImuDisplacement::ImuDisplacement()
    : imu_displacement_(0),
      imu_velocity_(0),
      previous_timestamp_micros_(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count())
{
}

void ImuDisplacement::updateImuDisplacement(const core::Float imu_acceleration)
{
  const core::TimePoint time_now = std::chrono::system_clock::now();
  const std::int64_t time_now_micros
    = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch()).count();
  const std::int64_t time_elapsed_seconds
    = static_cast<double>((time_now_micros - previous_timestamp_micros_) / 1'000'000);

  // from equation v=u+at
  const core::Float velocity_estimate = imu_velocity_ + (imu_acceleration * time_elapsed_seconds);

  // from equation s = ut + 0.5*a*(t^2)
  imu_displacement_ = (imu_velocity_ * time_elapsed_seconds)
                      + (0.5 * imu_acceleration * std::pow(time_elapsed_seconds, 2));

  // update class members
  imu_velocity_              = velocity_estimate;
  previous_timestamp_micros_ = time_now_micros;
}

core::Float ImuDisplacement::getImuDisplacement()
{
  // TODO: error handling
  return imu_displacement_;
}

core::Float ImuDisplacement::getImuVelocity()
{
  // TODO: error handling
  return imu_velocity_;
}

}  // namespace hyped::navigation