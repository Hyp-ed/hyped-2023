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
    // TODO : change magnitude for directions
    core::Float magnitude;
    for (std::size_t i = 0; i < core::kNumImus; ++i)
    {
      magnitude = 0;
      for (std::size_t j = 0; j < 3; ++j)
      {
        magnitude += std::pow(imu_data.at(i).at(j), 2);
      }
      clean_accelerometer_data.at(i) = std::sqrt(magnitude);
    }
      std::array<core::Float, 3> data_bounds = getOutlierThresholds(clean_accelerometer_data);
      core::Float lower_bound = data_bounds.at(0);
      core::Float upper_bound = data_bounds.at(1);
      core::Float median = data_bounds.at(2);
      for (size_t i = 0; i < core::kNumImus; ++i) {
        if (are_imus_reliable_.at(i) == false){
          clean_accelerometer_data.at(i) = median;
        } else if (clean_accelerometer_data.at(i) < lower_bound || clean_accelerometer_data.at(i) > upper_bound) {
          clean_accelerometer_data.at(i) = median;
          ++num_outliers_per_imu_.at(i);
        } else {
          num_outliers_per_imu_.at(i) = 0;
        }
      }
    return clean_accelerometer_data;
  }

  std::array<core::Float, 3> ImuPreprocessor::getOutlierThresholds(const core::ImuData &clean_accelerometer_data){
    /*
    TODO write function to get outliers:
      - test if a sensor is unreliable and copy data
      - sort copy of array
      - find q1, median, q3 and iqr
      - find upper and lower bounds 
      - return upper and lower bounds and median
    */ 
   std::array<core::Float, 3> data_bounds;
   const uint8_t num_reliable_accelerometers = std::accumulate(are_imus_reliable_.begin(), are_imus_reliable_.end(), 0);

   if (num_reliable_accelerometers == 4) {
      core::ImuData clean_accelerometer_data_copy = std::copy(clean_accelerometer_data.begin(), clean_accelerometer_data.end());
      std::sort(clean_accelerometer_data_copy.begin(), clean_accelerometer_data_copy.end());
      const core::Float q1 = (clean_accelerometer_data_copy.at(0) + clean_accelerometer_data_copy.at(1)) / 2.0;
      const core::Float median = (clean_accelerometer_data_copy.at(1) + clean_accelerometer_data_copy.at(2)) / 2.0;
      const core::Float q3 = (clean_accelerometer_data_copy.at(2) + clean_accelerometer_data_copy.at(3)) / 2.0;
      const core::Float iqr = q3 - q1;
      data_bounds.at(0) = q1 - 1.5*iqr;
      data_bounds.at(1) = q3 + 1.5*iqr;
      data_bounds.at(2) = median;

   } else if (num_reliable_accelerometers == 3) {
      std::array<core::Float, 3> clean_accelerometer_data_copy;
      int index = std::find(are_imus_reliable.begin(), are_imus_reliable.end(), false);
      for (size_t i = 0; i < 3; ++i){
        if (i >= index) {
          clean_accelerometer_data_copy.at(i) = clean_accelerometer_data.at(i + 1);
        } else {
          clean_accelerometer_data_copy.at(i) = clean_accelerometer_data.at(i);
        }
      }

      std::sort(clean_accelerometer_data_copy.begin(), clean_accelerometer_data_copy.end())
      const core::Float q1 = (clean_accelerometer_data_copy.at(0) + clean_accelerometer_data_copy.at(1)) / 2.0;
      const core::Float median = (clean_accelerometer_data_copy.at(1)) / 2.0;
      const core::Float q3 = (clean_accelerometer_data_copy.at(1) + clean_accelerometer_data_copy.at(2)) / 2.0;
      const core::Float iqr = q3 - q1;
      data_bounds.at(0) = q1 - 1.5*iqr;
      data_bounds.at(1) = q3 + 1.5*iqr;
      data_bounds.at(2) = median;

    } 

      return data_bounds;
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