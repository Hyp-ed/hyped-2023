#pragma once

#include "core/types.hpp"
#include <navigation/control/navigator.hpp>

namespace hyped::utils {

class NaiveNavigator : public navigation::INavigator {
 public:
  NaiveNavigator();
  virtual std::optional<core::Trajectory> currentTrajectory();
  virtual void keyenceUpdate(const core::KeyenceData &keyence_data);
  virtual void encoderUpdate(const core::EncoderData &encoder_data);
  virtual void accelerometerUpdate(const core::RawAccelerometerData &accelerometer_data);

 private:
  core::Trajectory current_trajectory_;
};

}  // namespace hyped::utils
