#pragma once

#include <cstdint>
#include <optional>

#include "core/time.hpp"
#include "core/timer.hpp"
#include "core/types.hpp"

namespace hyped::navigation {

class ImuDisplacement {
 public:
  ImuDisplacement(const core::ITimeSource &time);

  /**
   * @brief update the imu estimate for both displacement and velocity. Displacement is
   * used for cross checking agreement with wheel encooders, velocity is used for us to
   * estimate velocity in the case we have no direct velocity sensors.
   *
   * @param imu_acceleration kalman filtered single value for acceleration
   */
  void updateImuDisplacement(const core::Float imu_acceleration);

  /**
   * @brief Get the Imu Displacement object
   *
   * @return imu estimate of acceleration
   */
  core::Float getImuDisplacement();

  /**
   * @brief Get the Imu Velocity object
   *
   * @return imu estimate of velocity
   */
  core::Float getImuVelocity();

 private:
  // imu estimate of acceleration
  core::Float imu_displacement_;

  // imu estimate of velocity
  core::Float imu_velocity_;

  // timestamp of the last measured acceleration value
  core::TimePoint previous_timestamp_;

  // time source
  const core::ITimeSource &time_;
};

}  // namespace hyped::navigation