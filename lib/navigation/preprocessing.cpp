#include "preprocessing.hpp"

namespace hyped::navigation {

Preprocessing::Preprocessing()
{
  // TODO: implement
}

void Preprocessing::imuOutlierDetection()
{
  /*
  TODO: implement
  rough process:
  - get q1, median, q3 of imu array
  - define upper & lower bounds as (-)1.5*inter-quatrile range
  - if any datapoint outwith this, set
  outlier_imus_[i] += 1
  -set outlier points as median

  -also has to be able to handle 1 unreliable sensor
  -also figure out return type/ what we update and update
  documentation as appropriate
  */
}

void Preprocessing::encoderOutlierDetection()
{
  /*
  TODO: implement
  rough process:
  - get q1, median, q3 of encoder array
  - define upper & lower bounds as (-)1.5*inter-quatrile range
  - if any datapoint outwith this, set
  outlier_encoders_[i] += 1
  -set outlier points as median

  -also has to be able to handle 1 unreliable sensor
  -also figure out return type/ what we update and update
  documentation as appropriate
  */
}

void Preprocessing::checkImusReliable()
{
  /*
  TODO: implement
  rough process:
  - check how many times an individual imu
  has been an outlier in a row (outlier imus)
  -if outlier_imus_[i] > n (tbd), mark imu as unreliable
  in reliable imus.
  -if number unreliable in reliable_imus > 1, fail state

  -also figure out return type/ what we update and update
  documentation as appropriate
  */
}

void Preprocessing::checkEncodersReliable()
{
  /*
  TODO: implement
  rough process:
  - check how many times an individual encoder
  has been an outlier in a row (outlier encoders)
  -if outlier_encoders_[i] > n (tbd), mark imu as unreliable
  in reliable imus.
  -if number unreliable in reliable_imus > 1, fail state

  -also figure out return type/ what we update and update
  documentation as appropriate
  */
}

core::KeyenceData Preprocessing::preprocessKeyence(const core::KeyenceData keyence_data)
{
  /*
  TODO: implement
  rough process:
  - set as class member (?)
  - check if keyence sensors agree
  - if they agree, set keyence disgreeemnt to false
  - if they disagree and keyenece_disagreement = false, set to true
  - if they disagree and keyenece_disagreement = true,
  we have 2 disgeements in a row so fail state
  - update with largest of sensor values

  -also figure out return type/ what we update and update
  documentation as appropriate
  */
  return {0, 0};
}

core::ImuData Preprocessing::preprocessImus(const core::RawImuData imu_data)
{
  /*
  TODO: implement
  rough plan:
  - set raw data as class member
  - assign imu_data to class object
  - run imu outlier detection
  - run check imus reliable
  - get mean and run kalman filter
  - update trajectory with mean

  -also figure out return type/ what we update and update
  documentation as appropriate
  */
  return {0.0, 0.0, 0.0, 0.0};
}

core::EncoderData Preprocessing::preprocessEncoders(const core::EncoderData encoder_data)
{
  /*
  TODO: implement
  rough plan:

  - assign encoder data to class object
  - run encoder outlier detection
  - run check encoders reliable
  - update trajectory with mean

  -also figure out return type/ what we update and update
  documentation as appropriate
  */
  return {0, 0, 0, 0};
}
}  // namespace hyped::navigation