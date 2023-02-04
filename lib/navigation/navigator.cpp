#include "navigator.hpp"

#include <cstdint>

#include "core/time.hpp"

namespace hyped::navigation {

Navigator::Navigator()
    : previous_encoder_reading_(core::timePointFromSecondsSinceEpoch(0), {0, 0, 0}),
      previous_keyence_reading_(core::timePointFromSecondsSinceEpoch(0), {0, 0})
{
  // TODOLater: impement
}

std::optional<core::Trajectory> Navigator::currentTrajectory()
{
  /*
  TODOLater: call cross-checker now to ensure return
  trajectory is accurate and delicious.

  Cross checker will (until camera is up and running) have
  to add integrated/differentiated value for velocity for now
  */
  return trajectory_;
}

void Navigator::keyenceUpdate(const core::RawKeyenceData &keyence_data)
{
  /*
  TODOLater:
  - ensure keyence data is strictly increasing
  - run preprocessing on keyence data (basically an agreement check)
  */
}

void Navigator::encoderUpdate(const core::RawEncoderData &encoder_data)
{
  /*
  TODOLater:
  - ensure encoder data is strictly increasing
  - run preprocessing
  - update trajectory_.displacement with mean of processed data
  */
}

void Navigator::accelerometerUpdate(const core::RawAccelerometerData &accelerometer_data)
{
  /*
  TODOLater:
  - run preprocessing
  - update trajectory_.acceleration with kalman estimate
  */
}

}  // namespace hyped::navigation
