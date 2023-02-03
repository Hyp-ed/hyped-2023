#pragma once

#include "consts.hpp"

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
  void keyenceUpdate(const core::KeyenceData &keyence_data);
  /**
   * @brief preprocesses encoder data and updates trajectory
   *
   * @param encoder_data
   */
  void encoderUpdate(const core::EncoderData &encoder_data);
  /**
   * @brief preprocesses accelerometer data and updates trajectory
   *
   * @param accelerometer_data
   */
  void accelerometerUpdate(const core::RawAccelerometerData &accelerometer_data);

 private:
  // previous readings
  core::EncoderData previous_encoder_reading_;
  core::KeyenceData previous_keyence_reading_;

  // current navigation trajectory
  core::Trajectory trajectory_;
};

}  // namespace hyped::navigation