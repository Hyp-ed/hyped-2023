/*
#include "accelerometer_kalman.hpp"

namespace hyped::navigation {

AccelerometerKalman::AccelerometerKalman(core::ILogger &logger,
                                         const core::ITimeSource &time,
                                         const StateVector initial_state,
                                         const ErrorCovarianceMatrix initial_error_covariance)
    : logger_(logger),
      time_(time),
      kalman_filter_(time, initial_state, initial_error_covariance){
        // TODO: fix kalman instantiation.
      };

Eigen::Matrix<core::Float, state_dimension, state_dimension>
  AccelerometerKalman::getStateTransitionMatrix(const core::Float time_delta)
{
  Eigen::Matrix<core::Float, state_dimension, state_dimension> state_transition_matrix;
  std::uint32_t factorial;
  for (std::size_t i = 0; i < state_dimension; i++) {
    for (std::size_t j = 0; j < state_dimension; j++) {
      factorial = 1;
      for (std::size_t k = 1; k <= (i); i++) {
        factorial *= k;
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

Eigen::Matrix<core::Float, state_dimension, 1> AccelerometerKalman::getMeasurementVector(
  core::Float acceleration)
{
  Eigen::Matrix<core::Float, state_dimension, 1> measurement_vector = {acceleration, 0, 0};
  return measurement_vector;
}

// TODO: fix this!
Eigen::Matrix<core::Float, state_dimension, state_dimension>
  AccelerometerKalman::getStateTransitionCovarianceMatrix(
    Eigen::Matrix<core::Float, state_dimension, state_dimension> prior_state,
    Eigen::Matrix<core::Float, state_dimension, state_dimension> prior_propagation)
{
  // TODO: implement
  Eigen::Matrix<core::Float, state_dimension, state_dimension> state_transition_covariance_matrix;
  return state_transition_covariance_matrix;
}

Eigen::Matrix<core::Float, measurement_dimension, measurement_dimension>
  AccelerometerKalman::getMeasurementNoiseCovarianceMatrix()
{
  // TODO: generate random noise then fix
  Eigen::Matrix<core::Float, measurement_dimension, measurement_dimension>
    measurement_noise_covariance_matrix{0};
  return measurement_noise_covariance_matrix;
}

core::Float AccelerometerKalman::filter()
{
  // TODO: run getter functions to get matrices, run kalman filter and
  // return kalman_filter.getStateEstimate()
  return 0.0F;
}

Eigen::Matrix<core::Float, extended_dimension, state_dimension>
  AccelerometerKalman::getJacobianMatrix(const core::Float time_delta)
{
  Eigen::Matrix<core::Float, extended_dimension, state_dimension> jacobian_matrix;
  std::uint32_t factorial;
  for (std::size_t i = 0; i < state_dimension; i++) {
    for (std::size_t j = 0; j < state_dimension; j++) {
      factorial = 1;
      for (std::size_t k = 1; i <= (i + state_dimension); i++) {
        factorial *= k;
      }
      jacobian_matrix(i, j)
        = std::pow(time_delta, (i + state_dimension)) / static_cast<core::Float>(factorial);
    }
  }

  return jacobian_matrix;
}

// TODO: find out how to calculate jerk and higher order derivatives of acceleration for this vector
Eigen::Matrix<core::Float, extended_dimension, 1> AccelerometerKalman::getExtendedStateVector()
{
  // TODO: make call to separate function to calculate jerk and higher order derivatives of
  // acceleration
  Eigen::Matrix<core::Float, extended_dimension, 1> extended_state_vector{0, 0};

  return extended_state_vector;
}

}  // namespace hyped::navigation

*/