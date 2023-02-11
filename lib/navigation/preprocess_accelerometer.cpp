#include "preprocess_accelerometer.hpp"

#include <core/types.hpp>

namespace hyped::navigation {

AccelerometerPreprocessor::AccelerometerPreprocessor(core::ILogger &logger)
    : logger_(logger),
      num_outliers_per_accelerometer_({0, 0, 0, 0}),
      are_accelerometers_reliable_({true, true, true, true}),
      num_reliable_accelerometers_(core::kNumAccelerometers)
{
}

std::optional<AccelerometerData> AccelerometerPreprocessor::processData(
  const core::CombinedRawAccelerometerData &raw_accelerometer_data)
{
  std::array<core::Float, core::kNumAccelerometers> accelerometer_data;
  // TODOLater : check on direction of travel
  for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
    const auto raw_acceleration = raw_accelerometer_data.value.at(i);
    std::uint64_t magnitude     = 0;
    magnitude += raw_acceleration.x * raw_acceleration.x;
    magnitude += raw_acceleration.y * raw_acceleration.y;
    magnitude += raw_acceleration.z * raw_acceleration.z;
    accelerometer_data.at(i) = std::sqrt(magnitude);
  }
  const auto sanitised_accelerometer_data
    = detectOutliers(AccelerometerData(raw_accelerometer_data.measured_at, accelerometer_data));
  if (checkReliable() == SensorDisagreement::kUnacceptable) { return std::nullopt; }
  return sanitised_accelerometer_data;
}

AccelerometerData AccelerometerPreprocessor::detectOutliers(AccelerometerData accelerometer_data)
{
  Quartiles quartiles;
  if (num_reliable_accelerometers_ == core::kNumAccelerometers) {
    quartiles = getQuartiles(accelerometer_data.value);
  } else if (num_reliable_accelerometers_ == core::kNumAccelerometers - 1) {
    std::array<core::Float, core::kNumAccelerometers - 1> filtered_data;
    std::size_t j = 0;
    for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
      if (are_accelerometers_reliable_.at(i)) {
        filtered_data.at(j) = accelerometer_data.value.at(i);
        ++j;
      }
    }
    quartiles = getQuartiles(filtered_data);
  } else {
    logger_.log(core::LogLevel::kFatal, "Maximum number of unreliable accelerometers exceeded");
  }
  // TODOLater : maybe make a nice data structure
  const core::Float iqr = quartiles.q3 - quartiles.q1;
  core::Float lower_bound;
  core::Float upper_bound;
  // TODOLater : Check these values
  if (num_reliable_accelerometers_ == core::kNumAccelerometers) {
    lower_bound = quartiles.median - 1.5 * iqr;
    upper_bound = quartiles.median + 1.5 * iqr;
  } else {
    lower_bound = quartiles.median - 1.2 * iqr;
    upper_bound = quartiles.median + 1.2 * iqr;
  }
  for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
    // converts outliers or unreliables to medians, updates number of consecutive outliers for each
    // sensor
    if (are_accelerometers_reliable_.at(i) == false) {
      accelerometer_data.value.at(i) = quartiles.median;
    } else if (accelerometer_data.value.at(i) < lower_bound
               || accelerometer_data.value.at(i) > upper_bound) {
      accelerometer_data.value.at(i) = quartiles.median;
      ++num_outliers_per_accelerometer_.at(i);
    } else {
      num_outliers_per_accelerometer_.at(i) = 0;
    }
  }
  return accelerometer_data;
}

SensorDisagreement AccelerometerPreprocessor::checkReliable()
{  // changes reliable sensor to false if max consecutive outliers are reached
  for (std::size_t i = 0; i < core::kNumAccelerometers; ++i) {
    if (are_accelerometers_reliable_.at(i) == true
        && num_outliers_per_accelerometer_.at(i) >= kNumAllowedAccelerometerFailures_) {
      are_accelerometers_reliable_.at(i) = false;
      num_reliable_accelerometers_ -= 1;
    }
  }
  if (num_reliable_accelerometers_ < core::kNumAccelerometers - 1) {
    logger_.log(core::LogLevel::kFatal, "Maximum number of unreliable accelerometers exceeded");
    return SensorDisagreement::kUnacceptable;
  }
  return SensorDisagreement::kAcceptable;
}

}  // namespace hyped::navigation
