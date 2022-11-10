#include "preprocess_imu.hpp"

#include <cmath>

#include <algorithm>
#include <iterator>
#include <numeric>

namespace hyped::navigation {

ImuPreprocessor::ImuPreprocessor()
{
  // TODOLater: implement
}

core::ImuData ImuPreprocessor::processData(const core::RawImuData raw_imu_data)
{
  core::ImuData clean_accelerometer_data;
  clean_accelerometer_data = detectOutliers(raw_imu_data);
  checkReliable();
  return clean_accelerometer_data;
}

core::ImuData ImuPreprocessor::detectOutliers(const core::RawImuData imu_data)
{
  core::ImuData clean_accelerometer_data;
  // TODOLater : change magnitude for directions (hardware configuration?)
  core::Float magnitude;
  for (std::size_t i = 0; i < core::kNumImus; ++i) {
    magnitude = 0;
    for (std::size_t j = 0; j < 3; ++j) {
      magnitude += std::pow(imu_data.at(i).at(j), 2);
    }
    clean_accelerometer_data.at(i) = std::sqrt(magnitude);
  }
  Quartiles quartiles     = getOutlierThresholds(clean_accelerometer_data);
  core::Float iqr         = quartiles.q3 - quartiles.q1;
  core::Float lower_bound = quartiles.median - 1.5 * iqr;
  core::Float upper_bound = quartiles.median + 1.5 * iqr;
  for (size_t i = 0; i < core::kNumImus; ++i) {
    // converts outliers or unreliables to medians, updates number of consecutive outliers for each
    // sensor
    if (are_imus_reliable_.at(i) == false) {
      clean_accelerometer_data.at(i) = quartiles.median;
    } else if (clean_accelerometer_data.at(i) < lower_bound
               || clean_accelerometer_data.at(i) > upper_bound) {
      clean_accelerometer_data.at(i) = quartiles.median;
      ++num_outliers_per_imu_.at(i);
    } else {
      num_outliers_per_imu_.at(i) = 0;
    }
  }
  return clean_accelerometer_data;
}

Quartiles ImuPreprocessor::getOutlierThresholds(const core::ImuData &clean_accelerometer_data)
{
  const uint8_t num_reliable_accelerometers
    = std::accumulate(are_imus_reliable_.begin(), are_imus_reliable_.end(), 0);

  if (num_reliable_accelerometers == core::kNumImus) {
    core::ImuData clean_accelerometer_data_copy;
    std::copy(clean_accelerometer_data.begin(),
              clean_accelerometer_data.end(),
              clean_accelerometer_data_copy.begin());
    std::sort(clean_accelerometer_data_copy.begin(), clean_accelerometer_data_copy.end());
    const core::Float q1
      = (clean_accelerometer_data_copy.at(0) + clean_accelerometer_data_copy.at(1)) / 2.0;
    const core::Float median
      = (clean_accelerometer_data_copy.at(1) + clean_accelerometer_data_copy.at(2)) / 2.0;
    const core::Float q3
      = (clean_accelerometer_data_copy.at(2) + clean_accelerometer_data_copy.at(3)) / 2.0;
    Quartiles quartiles;
    quartiles.q1     = q1;
    quartiles.median = median;
    quartiles.q3     = q3;
    return quartiles;
  }
  // if an unreliable sensor exists, copy true values into another array
  std::array<core::Float, core::kNumImus - 1> clean_accelerometer_data_copy;
  std::uint8_t index;
  for (size_t i = 0; i < core::kNumImus - 1; ++i) {
    if (!are_imus_reliable_.at(i)) { index = i; }
  }

  for (size_t i = 0; i < core::kNumImus - 1; ++i) {
    if (i >= index) {
      clean_accelerometer_data_copy.at(i) = clean_accelerometer_data.at(i + 1);
    } else {
      clean_accelerometer_data_copy.at(i) = clean_accelerometer_data.at(i);
    }
  }

  std::sort(clean_accelerometer_data_copy.begin(), clean_accelerometer_data_copy.end());
  const core::Float q1
    = (clean_accelerometer_data_copy.at(0) + clean_accelerometer_data_copy.at(1)) / 2.0;
  const core::Float median = (clean_accelerometer_data_copy.at(1)) / 2.0;
  const core::Float q3
    = (clean_accelerometer_data_copy.at(1) + clean_accelerometer_data_copy.at(2)) / 2.0;
  Quartiles quartiles;
  quartiles.q1     = q1;
  quartiles.median = median;
  quartiles.q3     = q3;

  /*
  TODO : sort this mess

  const uint8_t num_reliable_accelerometers
    = std::accumulate(are_imus_reliable_.begin(), are_imus_reliable_.end(), 0);
  if (num_reliable_accelerometers == core::kNumImus){
    core::ImuData clean_accelerometer_data_copy;
    std::copy(clean_accelerometer_data.begin(),
              clean_accelerometer_data.end(),
              clean_accelerometer_data_copy.begin());
  } else {
    std::array<core::Float, core::kNumImus - 1> clean_accelerometer_data_copy;
    std::uint8_t index;
    for (size_t i = 0; i < core::kNumImus - 1; ++i) {
      if (!are_imus_reliable_.at(i)) { index = i; }
  }

    for (size_t i = 0; i < core::kNumImus - 1; ++i) {
      if (i >= index) {
        clean_accelerometer_data_copy.at(i) = clean_accelerometer_data.at(i + 1);
      } else {
        clean_accelerometer_data_copy.at(i) = clean_accelerometer_data.at(i);
    }
  }
  }
  Quartiles quartiles;
  if (clean_accelerometer_data_copy.size() % 2 == 0){
    quartiles.median =  ((clean_accelerometer_data_copy.at(clean_accelerometer_data_copy.size()/2) +
  clean_accelerometer_data_copy.at((clean_accelerometer_data_copy.size()/2) + 1))/2.0);
  }
  */

  return quartiles;
}

void ImuPreprocessor::checkReliable()
{
  std::uint8_t num_unreliable = 0;
  num_unreliable
    = core::kNumImus - std::accumulate(are_imus_reliable_.begin(), are_imus_reliable_.end(), 0);
  // changes reliable sensor to false if max consecutive outliers are reached
  for (size_t i = 0; i < core::kNumImus; ++i) {
    if (num_outliers_per_imu_.at(i) >= kNumAllowedImuFailures_) {
      are_imus_reliable_.at(i) = false;
    }
  }
  if (num_unreliable > 1) {
    // TODOLater : Fail State implementation
  }
}

}  // namespace hyped::navigation