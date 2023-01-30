#pragma once

#include <cstdint>
#include <optional>

#include "core/time.hpp"
#include "core/timer.hpp"
#include "core/types.hpp"

namespace hyped::navigation {

class AccelerometerTrajectoryEstimator {
 public:
  AccelerometerTrajectoryEstimator(const core::ITimeSource &time,
                                   const core::TimePoint initial_time);
  /**
   * @brief update the imu estimate for both displacement and velocity. Displacement is
   * used for cross checking agreement with wheel encooders, velocity is used for us to
   * estimate velocity in the case we have no direct velocity sensors.
   *
   * @param imu_acceleration kalman filtered single value for acceleration
   * @param imu_timestamp time at which easurement was taken
   */
  void update(const core::Float imu_acceleration, const core::TimePoint imu_timestamp);

  core::Float getAccelerometerDisplacement() const;

  core::Float getAccelerometerVelocity() const;

 private:
  core::Float imu_displacement_;
  core::Float imu_velocity_;
  core::TimePoint previous_timestamp_;
  const core::ITimeSource &time_;
};

}  // namespace hyped::navigation