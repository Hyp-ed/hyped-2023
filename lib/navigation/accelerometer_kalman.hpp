#pragma once

#include "consts.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

#include <Eigen/Dense>
#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>

namespace hyped::navigation {
class AccelerometerKalman {
 public:
  AccelerometerPreprocessor(core::ILogger &logger, const core::ITimeSource &time);

  /**
   * @brief convert raw accelerometer data to cleaned and filtered data
   *
   * @param raw_accelerometer_data
   * @return clean accelerometer data or optionally fail
   */

  static constexpr std::size_t num_states_ = 3;

 private:
  core::ILogger &logger_;
  const core::ITimeSource &time_;
  // Kalman Filter variables

  const Eigen::Matrix<core::Float, 1, num_states_> measurement_matrix_ = {1, 0, 0};

  // TODO: implement and document these functions
  Eigen::Matrix<core::Float, num_states_, num_states_> getMeasurementMatrix(
    core::Float acceleration);
  Eigen::Matrix<core::Float, num_states_, num_states_> getStateTransitionMatrix(
    const core::Duration time_delta);
  Eigen::Matrix<core::Float, num_states_, num_states_> getStateTransitionCovarianceMatrix();
  Eigen::Matrix<core::Float, 1, 1> getMeasurementNoiseCovarianceMatrix();
};

}  // namespace hyped::navigation
