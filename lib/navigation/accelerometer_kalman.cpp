#include "accelerometer_kalman.hpp"

#include <core/types.hpp>

namespace hyped::navigation {

AccelerometerKalman::AccelerometerKalman(core::ILogger &logger, const core::ITimeSource &time)
    : logger_(logger),
      time_(time){
        // TODO: instantiate kalman

      };

Eigen::
  Matrix<core::Float, AccelerometerKalman::state_dimension_, AccelerometerKalman::state_dimension_>
  AccelerometerKalman::getStateTransitionMatrix(const core::Float time_delta)
{
  Eigen::Matrix<core::Float,
                AccelerometerKalman::state_dimension_,
                AccelerometerKalman::state_dimension_>
    state_transition_matrix;

  for (std::size_t i = 0; i < AccelerometerKalman::state_dimension_; i++) {
    for (std::size_t j = 0; j < AccelerometerKalman::state_dimension_; j++) {
      std::uint64_t factorial = 1;
      for (std::size_t i = 1; i <= (i); i++) {
        factorial *= i;
      }
      if (j - i > 0) {
        state_transition_matrix(i, j) = 0;
        continue;
      }
      state_transition_matrix(i, j)
        = std::pow(time_delta, (i)) / static_cast<core::Float>(factorial);
    }
  }
  return state_transition_matrix;
}

Eigen::Matrix<core::Float, AccelerometerKalman::state_dimension_, 1>
  AccelerometerKalman::getMeasurementVector(core::Float acceleration)
{
  Eigen::Matrix<core::Float, AccelerometerKalman::state_dimension_, 1> measurement_vector
    = {acceleration, 0, 0};
  return measurement_vector;
}

Eigen::
  Matrix<core::Float, AccelerometerKalman::state_dimension_, AccelerometerKalman::state_dimension_>
  AccelerometerKalman::getStateTransitionCovarianceMatrix()
{
  // TODO: implement
  Eigen::Matrix<core::Float,
                AccelerometerKalman::state_dimension_,
                AccelerometerKalman::state_dimension_>
    state_transition_covariance_matrix;
  return state_transition_covariance_matrix;
}

Eigen::Matrix<core::Float,
              AccelerometerKalman::measurement_dimension_,
              AccelerometerKalman::measurement_dimension_>
  AccelerometerKalman::getMeasurementNoiseCovarianceMatrix()
{
  // TODO: implement
  Eigen::Matrix<core::Float,
                AccelerometerKalman::measurement_dimension_,
                AccelerometerKalman::measurement_dimension_>
    measurement_noise_covariance_matrix{0};
  return measurement_noise_covariance_matrix;
}

core::Float AccelerometerKalman::filter()
{
  // TODO: run getter functions to get matrices, run kalman filter and
  // return kalman_filter.getStateEstimate()
  return 0.0F;
}

Eigen::Matrix<core::Float,
              AccelerometerKalman::extended_dimension_,
              AccelerometerKalman::state_dimension_>
  AccelerometerKalman::getJacobianMatrix(const core::Float time_delta)
{
  Eigen::Matrix<core::Float,
                AccelerometerKalman::extended_dimension_,
                AccelerometerKalman::state_dimension_>
    jacobian_matrix;

  for (std::size_t i = 0; i < AccelerometerKalman::state_dimension_; i++) {
    for (std::size_t j = 0; j < AccelerometerKalman::state_dimension_; j++) {
      std::uint64_t factorial = 1;
      for (std::size_t i = 1; i <= (i + state_dimension_); i++) {
        factorial *= i;
      }
      jacobian_matrix(i, j)
        = std::pow(time_delta, (i + state_dimension_)) / static_cast<core::Float>(factorial);
    }
  }

  return jacobian_matrix;
}

// TODO: find out how to calculate jerk and higher order derivatives of acceleration for this vector
Eigen::Matrix<core::Float, AccelerometerKalman::extended_dimension_, 1>
  AccelerometerKalman::getExtendedStateVector()
{
  Eigen::Matrix<core::Float, AccelerometerKalman::extended_dimension_, 1> extended_state_vector{0,
                                                                                                0};

  return extended_state_vector;
}

}  // namespace hyped::navigation