#include "preprocess_imu.hpp"

#include <cmath>

namespace hyped::navigation
{

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
    core::ImuData clean_accelerometer_data;
    core::Float magnitude;
    for (size_t i = 0; i < core::kNumImus; ++i)
    {
      magnitude = 0;
      for (size_t j = 0; j < 3; ++j)
      {
        magnitude += std::pow(imu_data.at(i).at(j), 2);
      }
      clean_accelerometer_data.at(i) = std::sqrt(magnitude);
    }
    const uint8_t num_reliable_accelerometers = std::accumulate(are_imus_reliable_.begin(), are_imus_reliable_.end(), 0);
    if (num_reliable_accelerometers == 4)
    { 
      core::ImuData clean_accelerometer_data_copy = std::copy(clean_accelerometer_data);
      std::sort(clean_accelerometer_data_copy.begin(), clean_accelerometer_data_copy.end());
      const core::Float q1 = (clean_accelerometer_data_copy.at(0) + clean_accelerometer_data_copy.at(1)) / 2.0;
      const core::Float median = (clean_accelerometer_data_copy.at(1) + clean_accelerometer_data_copy.at(2)) / 2.0;
      const core::Float q3 = (clean_accelerometer_data_copy.at(2) + clean_accelerometer_data_copy.at(3)) / 2.0;
      const core::Float iqr = q3 - q1;

      for (int i = 0; i < core::kNumImus; ++i) {
        if (clean_accelerometer_data.at(i) > q3 + 1.5*iqr || clean_accelerometer_data.at(i) < q1 - 1.5*iqr) {
            clean_accelerometer_data.at(i) = median;
            num_outliers_per_imu_.at(i) += 1;
        } else {
          num_outliers_per_imu_.at(i) = 0;
        }
      }
    } else if (num_reliable_accelerometers == 3) {
      core::ImuData clean_accelerometer_data_copy = std::copy(clean_accelerometer_data);
      for (int i = 0; i < core::kNumImus; ++i) {
        if (are_imus_reliable_.at(i) == false) {
          clean_accelerometer_data_copy.at(i) = -1;
        }
      }
      std::sort(clean_accelerometer_data_copy.begin(), clean_accelerometer_data_copy.end());
      const core::Float q1 = (clean_accelerometer_data_copy.at(1) + clean_accelerometer_data_copy.at(2)) / 2.0;
      const core::Float median = clean_accelerator_data_copy.at(2);
      const core::Float q3 = (clean_accelerometer_data_copy.at(2) + clean_accelerometer_data_copy.at(3)) / 2.0;
      const core::Float iqr = q3 - q1;
      
      for (int i = 0; i < core::kNumImus; ++i) {
        if (are_imus_reliable_.at(i) == false) {
          clean_accelerometer_data.at(i) = median;
        } else {
          if (clean_accelerometer_data.at(i) > q3 + 1.5*iqr || clean_accelerometer_data.at(i) < q1 - 1.5*iqr) {
            clean_accelerometer_data.at(i) = median;
            num_outliers_per_imu_.at(i) += 1;
          } else {
            num_outliers_per_imu_.at(i) = 0;
          }
        }
      }
    } else {
      // TODO check Fail state implementation
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

} // namespace hyped::navigation