#pragma once

#include <core/time.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <navigation/types.hpp>

namespace hyped::utils {

class NaiveNavigator : public navigation::INavigator {
 public:
  NaiveNavigator();
  virtual std::optional<core::Trajectory> currentTrajectory();
  virtual void keyenceUpdate(const core::RawKeyenceData &keyence_data);
  virtual void encoderUpdate(const core::RawEncoderData &encoder_data);
  virtual void accelerometerUpdate(const core::CombinedRawAccelerometerData &accelerometer_data);

 private:
  core::Trajectory current_trajectory_;
  core::TimePoint last_encoder_update_;
};

}  // namespace hyped::utils
