#include "preprocess_imu.hpp"

#include <cmath>

#include <algorithm>
#include <numeric>
#include <optional>

namespace hyped::navigation {

ImuPreprocessor::ImuPreprocessor(core::ILogger &logger)
    : log_(logger),
      num_outliers_per_imu_({0, 0, 0, 0}),
      are_imus_reliable_({true, true, true, true}),
      num_reliable_accelerometers_(core::kNumImus)
{
}

std::optional<core::ImuData> ImuPreprocessor::processData(const core::RawImuData raw_imu_data)
{
  core::ImuData imu_data;
  // TODOLater : check on direction of travel
  core::Float magnitude;
  for (std::size_t i = 0; i < core::kNumImus; ++i) {
    magnitude = 0;
    for (std::size_t j = 0; j < 3; ++j) {
      magnitude += std::pow(raw_imu_data.at(i).at(j), 2);
    }
    imu_data.at(i) = std::sqrt(magnitude);
  }

  const core::ImuData accelerometer_data = detectOutliers(imu_data);
  checkReliable();
  return accelerometer_data;
}

core::ImuData ImuPreprocessor::detectOutliers(const core::ImuData imu_data)
{
  core::ImuData accelerometer_data;
  for (std::size_t i; i < core::kNumImus; ++i) {
    accelerometer_data.at(i) = imu_data.at(i);
  }

  Quartiles quartiles;
  if (num_reliable_accelerometers_ == core::kNumImus) {
    quartiles = getQuartiles(imu_data);
  } else {
    std::array<core::Float, core::kNumImus - 1> faulty_data;
    std::size_t index;
    for (std::size_t i = 0; i < core::kNumImus - 1; ++i) {
      if (!are_imus_reliable_.at(i)) { index = i; }
    }
    for (std::size_t i = 0; i < core::kNumImus - 1; ++i) {
      if (i >= index) {
        faulty_data.at(i) = imu_data.at(i + 1);
      } else {
        faulty_data.at(i) = imu_data.at(i);
      }
    }
    quartiles = getQuartiles(faulty_data);
  }

  const core::Float iqr = quartiles.q3 - quartiles.q1;
  core::Float lower_bound;
  core::Float upper_bound;

  // TODOLater : Check these values
  if (num_reliable_accelerometers_ == core::kNumImus) {
    lower_bound = quartiles.median - 1.5 * iqr;
    upper_bound = quartiles.median + 1.5 * iqr;
  } else {
    lower_bound = quartiles.median - 1.2 * iqr;
    upper_bound = quartiles.median + 1.2 * iqr;
  }

  for (size_t i = 0; i < core::kNumImus; ++i) {
    // converts outliers or unreliables to medians, updates number of consecutive outliers for each
    // sensor
    if (are_imus_reliable_.at(i) == false) {
      accelerometer_data.at(i) = quartiles.median;
    } else if (imu_data.at(i) < lower_bound || imu_data.at(i) > upper_bound) {
      accelerometer_data.at(i) = quartiles.median;
      ++num_outliers_per_imu_.at(i);
    } else {
      num_outliers_per_imu_.at(i) = 0;
    }
  }
  return accelerometer_data;
}

template<std::size_t N>
Quartiles ImuPreprocessor::getQuartiles(const std::array<core::Float, N> &clean_accelerometer_data)
{
  std::array<core::Float, N> accelerometer_data_copy;
  std::copy(clean_accelerometer_data.begin(),
            clean_accelerometer_data.end(),
            accelerometer_data_copy.begin());

  std::sort(accelerometer_data_copy.begin(), accelerometer_data_copy.end());

  Quartiles quartiles;
  quartiles.q1     = getSpecificQuartile(accelerometer_data_copy, 0.25);
  quartiles.median = getSpecificQuartile(accelerometer_data_copy, 0.5);
  quartiles.q3     = getSpecificQuartile(accelerometer_data_copy, 0.75);
  return quartiles;
}

template<std::size_t N>
core::Float ImuPreprocessor::getSpecificQuartile(
  const std::array<core::Float, N> &clean_accelerometer_data, core::Float quartile)
{
  const core::Float index_quartile       = (num_reliable_accelerometers_ - 1) * quartile;
  const std::uint8_t index_quartile_high = static_cast<int>(std::ceil(index_quartile));
  const std::uint8_t index_quartile_low  = static_cast<int>(std::floor(index_quartile));
  const core::Float quartile_value       = (clean_accelerometer_data.at(index_quartile_high)
                                      + clean_accelerometer_data.at(index_quartile_low))
                                     / 2.0;
  return quartile_value;
}

SensorChecks ImuPreprocessor::checkReliable()
{  // changes reliable sensor to false if max consecutive outliers are reached
  for (std::size_t i = 0; i < core::kNumImus; ++i) {
    if (are_imus_reliable_.at(i) == true
        && num_outliers_per_imu_.at(i) >= kNumAllowedImuFailures_) {
      are_imus_reliable_.at(i) = false;
      num_reliable_accelerometers_ -= 1;
    }
  }
  if (num_reliable_accelerometers_ < core::kNumImus - 1) {
    log_.log(core::LogLevel::kFatal, "Maximum number of unreliable sensors exceeded");
    return SensorChecks::kUnacceptable;
  }
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation