#pragma once
#include "types.hpp"

#include <optional>

#include "core/types.hpp"

namespace hyped::navigation {

class Navigator : public INavigator {
 public:
  Navigator();

  /**
   *@brief runs cross checking and returns trajectory
   */
  std::optional<core::Trajectory> currentTrajectory();

  /**
   * @brief preprocesses keyence data and updates trajectory
   *
   * @param keyence_data
   */
  void keyenceUpdate(const core::RawKeyenceData &keyence_data);
  /**
   * @brief preprocesses encoder data and updates trajectory
   *
   * @param encoder_data
   */
  void encoderUpdate(const core::RawEncoderData &encoder_data);
  /**
   * @brief preprocesses imu data and updates trajectory
   *
   * @param acceleration_data
   */
  void accelerometerUpdate(const core::RawAccelerationData &acceleration_data);

 private:
  // previous readings
  core::RawEncoderData previous_encoder_reading_;
  core::RawKeyenceData previous_keyence_reading_;

  // current navigation trajectory
  core::Trajectory trajectory_;
};

}  // namespace hyped::navigation
