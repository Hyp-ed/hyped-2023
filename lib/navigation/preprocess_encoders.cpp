#include "preprocess_encoders.hpp"

namespace hyped::navigation {

EncodersPreprocessor::EncodersPreprocessor()
{
  // TODOLater: implement
}

std::optional<EncoderData> EncodersPreprocessor::processData(
  const core::RawEncoderData &raw_encoder_data)
{
  /*
  TODOLater: implement
  basic plan:
  - call detectOutliers. Return of that function is return of this function
  - call checkRelaible

  */
  return std::nullopt;
}

EncoderData EncodersPreprocessor::detectOutliers(const core::RawEncoderData &raw_encoder_data)
{
  /*
  TODOLater: implement
  rough process:
  - get q1, median, q3 of encoder array
  - define upper & lower bounds as (-)1.5*inter-quatrile range
  - if any datapoint outwith this, set
  num_outliers_per_encoder_[i] += 1
  -set outlier points as median

  -also has to be able to handle 1 unreliable sensor
  -also figure out return type/ what we update and update
  documentation as appropriate
  */
  return {0, 0, 0, 0};
}

void EncodersPreprocessor::checkReliable(const core::RawEncoderData &raw_encoder_data)
{
  /*
  TODOLater: implement
  rough process:
  - check how many times an individual encoder
  has been an outlier in a row (outlier encoders)
  -if num_outliers_per_encoder_[i] > n (tbd), mark encoder as unreliable
  in reliable encoders.
  -if number unreliable in reliable_encoders > 1, fail state

  -also figure out return type/ what we update and update
  documentation as appropriate
  */
}

}  // namespace hyped::navigation
