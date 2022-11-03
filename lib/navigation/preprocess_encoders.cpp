#include "preprocess_encoders.hpp"

namespace hyped::navigation {

PreprocessEncoders::PreprocessEncoders()
{
  // TODO: implement
}

core::EncoderData PreprocessEncoders::processData(const core::EncoderData)
{
  /*
  TODO: implement
  basic plan:
  - call encodersOutlierDetection. Return of that function is return of this function
  - call checkEncodersRelaible

  */
  return {0, 0, 0, 0};
}

core::EncoderData PreprocessEncoders::encodersOutlierDetection(const core::EncoderData encoder_data)
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
  return {0, 0, 0, 0};
}

void PreprocessEncoders::checkEncodersReliable(const core::EncoderData encoder_data)
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

}  // namespace hyped::navigation