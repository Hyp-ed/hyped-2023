#include "crosscheck.hpp"

namespace hyped::navigation {

Crosscheck::Crosscheck()
{
  // TODOLater: implement
}

SensorChecks Crosscheck::checkTrajectoryAgreement(const core::AccelerometerData accelerometer_data,
                                                  const core::EncoderData encoders_data,
                                                  const core::KeyenceData keyence_data)
{
  /*
  TODOLater: implement
  basically:
    - checkEncoderAccelerometer
    - checkEncooderKeyence
    - if all good, return true. else false and fail state

    Also need to figure out how data flow is going to work with the historic data and what we use.
    The basic infrastrucutre is there for now so will be a problem for another day.
  */
  return SensorChecks::kAcceptable;
}

SensorChecks checkEncoderAccelerometer(const core::AccelerometerData accelerometer_data,
                                       const core::EncoderData encoders_data)
{
  /*
  TODOLater: implement.
  plan:
  - double integrate z accelerometer values (also TODOLater)
  - if absolute diff between encoder displacement and
  accelerometer displacement too high, fail state and return false
  - otherwise all good, return true
  */
  return SensorChecks::kAcceptable;
}

SensorChecks checkEncoderKeyence(const core::EncoderData encoder_data,
                                 const core::KeyenceData keyence_data)
{
  /*
  TODOLater: implement.
  plan:
  - if absolute diff between encoder displacement and
  keyence displacement too high, fail state and return false
  - otherwise all good, return true
  */
  return SensorChecks::kAcceptable;
}
}  // namespace hyped::navigation