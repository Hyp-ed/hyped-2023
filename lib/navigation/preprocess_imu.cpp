#include "preprocess_imu.hpp"

namespace hyped::navigation {

ImuPreprocessor::ImuPreprocessor(core::ILogger &logger)
    : logger_(logger),
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
    for (std::size_t j = 0; j < core::kNumAxis; ++j) {
      magnitude += std::pow(raw_imu_data.at(i).at(j), 2);
    }
    imu_data.at(i) = std::sqrt(magnitude);
  }
  const core::ImuData accelerometer_data = detectOutliers(imu_data);
  SensorChecks sensorcheck               = checkReliable();

  if (sensorcheck == SensorChecks::kUnacceptable) {
    return std::nullopt;
  } else {
    return accelerometer_data;
  }
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
  } else if (num_reliable_accelerometers_ == core::kNumImus - 1) {
    std::array<core::Float, core::kNumImus - 1> filtered_data;
    std::size_t j = 0;
    for (std::size_t i = 0; i < core::kNumImus; ++i) {
      if (are_imus_reliable_.at(i)) {
        filtered_data.at(j) = imu_data.at(i);
        ++j;
      }
    }
    quartiles = getQuartiles(filtered_data);
  } else {
    logger_.log(core::LogLevel::kFatal, "Maximum number of unreliable sensors exceeded");
  }
  // TODOLater : maybe make a nice data structure
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
    logger_.log(core::LogLevel::kFatal, "Maximum number of unreliable sensors exceeded");
    return SensorChecks::kUnacceptable;
  }
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation