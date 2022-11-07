#include "preprocess_imu.hpp"

namespace hyped::navigation {

ImuPreprocessor::ImuPreprocessor()
{
  // TODOLater: implement
}

core::ImuData ImuPreprocessor::processData(const core::RawImuData raw_imu_data)
{
  /*
  TODOLater: implement
  basic plan:
  - call detectOutliers. Return of that function is return of this function
  - call checkReliable

  */
  return {0, 0, 0, 0};
}

core::ImuData ImuPreprocessor::detectOutliers(const core::RawImuData imu_data)
{
  /*
  TODOLater: implement
  rough process:
  - get q1, median, q3 of encoder array
  - define upper & lower bounds as (-)1.5*inter-quatrile range
  - if any datapoint outwith this, set
  outlier_imus_[i] += 1
  -set outlier points as median

  -also has to be able to handle 1 unreliable sensor
  -also figure out return type/ what we update and update
  documentation as appropriate
  */
  const uint8_t num_reliable_imus = std::accumulate(are_imus_reliable_.start(), are_imus_reliable_.end(), 0);
  if (num_reliable_imus == 4) {
  std::sort(imu_data.start(), imu_data.end());
  const float q1 = (imu_data.at(0) + imu_data.at(1))/2.0;
  const float median = (imu_data.at(1) + imu_data.at(2))/2.0;
  const float q3 = (imu_data.at(2) + imu_data.at(3))/2.0;
  } 
  return {0, 0, 0, 0};
}

void ImuPreprocessor::checkReliable(const core::ImuData &imu_data)
{
  /*
  TODOLater: implement
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

}  // namespace hyped::navigation