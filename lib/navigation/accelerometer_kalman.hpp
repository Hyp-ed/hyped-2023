#pragma once

#include "consts.hpp"

#include <cstdint>

#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <eigen3/Eigen/Dense>
#include <extended_kalman_filter.hpp>

namespace hyped::navigation {
class AccelerometerKalman {
 public:
  AccelerometerKalman(core::ILogger &logger,
                      const core::ITimeSource &time,
                      const StateVector initial_state,
                      const ErrorCovarianceMatrix initial_error_covariance);

  /**
   * @brief convert raw accelerometer data to cleaned and filtered data
   *
   * @param raw_accelerometer_data
   * @return clean accelerometer data or optionally fail
   */

  core::Float filter();

  static constexpr std::size_t state_dimension_       = 3;  // TODO: change this!
  static constexpr std::size_t measurement_dimension_ = 1;
  static constexpr std::size_t extended_dimension_    = 2;  // TODO: check this!

 private:
  ExtendedKalmanFilter<3, 1, 2> kalman_filter_;

  core::ILogger &logger_;
  const core::ITimeSource &time_;
  // TODO: change for actual state vector
  Eigen::Matrix<core::Float, state_dimension_, 1> initial_state = {0, 0, 0};
  Eigen::Matrix<core::Float, state_dimension_, state_dimension_> initial_error_covariance;
  // TODO: make this less bad
  //{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  ExtendedKalmanFilter<state_dimension_, measurement_dimension_, extended_dimension_>
    kalman_filter_(
      const core::ITimeSource &time_,
      Eigen::Matrix<core::Float, state_dimension_, 1> initial_state,
      Eigen::Matrix<core::Float, state_dimension_, state_dimension_> initial_error_covariance);

  const Eigen::Matrix<core::Float, state_dimension_, state_dimension_> measurement_matrix_;
  //{{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  // TODO: implement and document these functions
  Eigen::Matrix<core::Float, state_dimension_, 1> getMeasurementVector(core::Float acceleration);
  Eigen::Matrix<core::Float, state_dimension_, state_dimension_> getStateTransitionMatrix(
    const core::Float time_delta);
  Eigen::Matrix<core::Float, state_dimension_, state_dimension_>
    getStateTransitionCovarianceMatrix();
  Eigen::Matrix<core::Float,
                AccelerometerKalman::measurement_dimension_,
                AccelerometerKalman::measurement_dimension_>
    getMeasurementNoiseCovarianceMatrix();
  Eigen::Matrix<core::Float, extended_dimension_, state_dimension_> getJacobianMatrix(
    const core::Float time_delta);
  Eigen::Matrix<core::Float, extended_dimension_, 1> getExtendedStateVector();
};

}  // namespace hyped::navigation
