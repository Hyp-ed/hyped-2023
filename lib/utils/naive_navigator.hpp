#pragma once

#include <core/time.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <navigation/types.hpp>

namespace hyped::utils {

class NaiveNavigator : public navigation::INavigator {
 public:
  NaiveNavigator(const core::ITimeSource &time_source);
  virtual std::optional<core::Trajectory> currentTrajectory();
  virtual void keyenceUpdate(const core::RawKeyenceData &keyence_data);
  virtual void encoderUpdate(const core::RawEncoderData &encoder_data);
  virtual void accelerometerUpdate(const core::RawAccelerometerData &accelerometer_data);

 private:
  const core::ITimeSource &time_source_;
  core::Trajectory current_trajectory_;
  core::Timer encoder_update_timer_;
};

}  // namespace hyped::utils
