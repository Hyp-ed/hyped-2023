#include "crosscheck.hpp"

namespace hyped::navigation {

Crosscheck::Crosscheck()
{
  // TODOLater: implement
}

SensorDisagreement Crosscheck::checkTrajectoryAgreement(const ImuData imu_data,
                                                        const EncoderData encoders_data,
                                                        const KeyenceData keyence_data)
{
  /*
  TODOLater: implement
  basically:
    - checkEncoderImu
    - checkEncooderKeyence
    - if all good, return true. else false and fail state

    Also need to figure out how data flow is going to work with the historic data and what we use.
    The basic infrastrucutre is there for now so will be a problem for another day.
  */
  return SensorDisagreement::kAcceptable;
}

SensorDisagreement checkEncoderImu(const ImuData imu_data, const EncoderData encoders_data)
{
  /*
  TODOLater: implement.
  plan:
  - double integrate imu values (also TODOLater)
  - if absolute diff between encoder displacement and
  imu displacement too high, fail state and return false
  - otherwise all good, return true
  */
  return SensorDisagreement::kAcceptable;
}

SensorDisagreement checkEncoderKeyence(const EncoderData encoder_data,
                                       const KeyenceData keyence_data)
{
  /*
  TODOLater: implement.
  plan:
  - if absolute diff between encoder displacement and
  keyence displacement too high, fail state and return false
  - otherwise all good, return true
  */
  return SensorDisagreement::kAcceptable;
}
}  // namespace hyped::navigation
