#include "navigator.hpp"

#include <cstdint>

namespace hyped::navigation {

Navigator::Navigator(core::ILogger &logger, const core::ITimeSource &time)
    : logger_(logger),
      time_(time),
      keyence_preprocessor_(logger),
      accelerometer_preprocessor_(logger, time)
{
  // TODOLater: implement, add log and timesource so far
  // TODOLater: instantiate everything?
}

std::optional<core::Trajectory> Navigator::currentTrajectory()
{
  /*
  TODOLater:
  - instantiate kalman
  - make all the arguents (store some as class data members?)
  - kalman.filter(*all the arguments*)
  - state_est = Kalman.getStateEstimate()
  - run cross checkers
  - if no oopsies, return trajectory

  TODOLater: call cross-checker now to ensure return
  trajectory is accurate and delicious.

  Cross checker will (until camera is up and running) have
  to add integrated/differentiated value for velocity for now
  */
  return trajectory_;
}

void Navigator::keyenceUpdate(const core::KeyenceData &keyence_data)
{
  // Check keyence strictly increasing
  if (keyence_data.at(0) < previous_keyence_reading_.at(0)) {
    logger_.log(core::LogLevel::kFatal, "Keyence data is decreasing");
  }

  // Run preprocessing on keyence and check result
  const SensorChecks keyence_check = keyence_preprocessor_.checkKeyenceAgrees(keyence_data);
  if (keyence_check == SensorChecks::kUnacceptable) {
    logger_.log(core::LogLevel::kFatal, "Keyence data has failed preprocessing");
  }

  // Update old keyence reading
  previous_keyence_reading_ = keyence_data;
  logger_.log(core::LogLevel::kInfo, "Keyence data successfully updated in Navigation");
}

void Navigator::encoderUpdate(const core::EncoderData &encoder_data)
{
  /*
  TODOLater:
  - ensure encoder data is strictly increasing
  - run preprocessing
  - update trajectory_.displacement with mean of processed data
  */
  for (std::size_t i = 0; i < core::kNumEncoders; ++i) {
    if (previous_encoder_reading_.at(i) < encoder_data.at(i)) {
      logger_.log(core::LogLevel::kFatal, "Encoder data is decreasing");
    }
  }
  // Still TODOLater: instnatiate and use encoders preprocessing
}

void Navigator::accelerometerUpdate(const core::RawAccelerometerData &accelerometer_data)
{
  // TODO: check accelerometer data structs consistent with what we're working with here
  auto processed_accelerometer_data = accelerometer_preprocessor_.processData(accelerometer_data);
  if (!processed_accelerometer_data) {
    logger_.log(core::LogLevel::kFatal, "Reliability error in accelerometer data");
  }
  // TODOLater filtering here!
  // TODO: update trajectory values with integrated acceelrometer data (so we have reliable
  // crosschecker)
}

}  // namespace hyped::navigation
