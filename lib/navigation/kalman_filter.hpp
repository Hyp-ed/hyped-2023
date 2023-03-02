#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <Eigen/Dense>
#include <core/time.hpp>
#include <core/types.hpp>

namespace hyped::navigation {

// State dimension 3, measurement dimension 1
template<std::size_t state_dimension, std::size_t measurement_dimension>
class KalmanFilter {
 public:
  // in order acc, velocity, displacement
  using StateVector           = Eigen::Matrix<core::Float, state_dimension, 1>;
  using StateTransitionMatrix = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
  using StateTransitionCovarianceMatrix
    = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
  using ErrorCovarianceMatrix = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
  using MeasurementMatrix
    = Eigen::Matrix<core::Float, measurement_dimension, state_dimension>;          //[1, 0, 0]
  using MeasurementVector = Eigen::Matrix<core::Float, measurement_dimension, 1>;  //[acc_val, 0, 0]
  using MeasurementNoiseCovarianceMatrix
    = Eigen::Matrix<core::Float, measurement_dimension, measurement_dimension>;

  KalmanFilter(std::shared_ptr<core::ITimeSource> time_source,
               const StateVector initial_state,
               const ErrorCovarianceMatrix initial_error_covariance)
      : time_source_(time_source),
        last_update_time_(time_source->now()),
        state_estimate_(initial_state),
        error_covariance_(initial_error_covariance)
  {
    static_assert(state_dimension > 0);
    static_assert(measurement_dimension > 0);
  }

  void filter(const StateTransitionMatrix &transition_matrix,
              const StateTransitionCovarianceMatrix &transition_covariance,
              const MeasurementMatrix &measurement_matrix,
              const MeasurementNoiseCovarianceMatrix &measurement_noise_covariance,
              const MeasurementVector &measurement)
  {
    // TODO: figure out how to make transition matrix given time delta - in main nav section
    const auto apriori_state_estimate = transition_matrix * state_estimate_;
    const auto apriori_error_covariance
      = (transition_matrix.transpose() * error_covariance_ * transition_matrix)
        + transition_covariance;
    // TODOLater: Some optimisation is to be found here:
    // 1. Try and get rid of the inverse.
    // 2. Reuse calculations such as `apriori_error_covariance * measurement_matrix.transpose()`.
    // K_k = P_k^{-1} * H_k^T * (H_k * P_k^{-1} * H_k^T + R_k)^{-1}
    const auto kalman_gain
      = apriori_error_covariance * measurement_matrix.transpose()
        * (measurement_matrix * apriori_error_covariance * measurement_matrix.transpose()
           + measurement_noise_covariance)
            .inverse();
    state_estimate_ = apriori_state_estimate
                      + kalman_gain * (measurement - measurement_matrix * apriori_state_estimate);
  }

  const StateVector &getStateEstimate() const { return state_estimate_; }
  const ErrorCovarianceMatrix &getErrorCovariance() const { return error_covariance_; }

 private:
  std::shared_ptr<core::ITimeSource> time_source_;
  core::TimePoint last_update_time_;
  StateVector state_estimate_;
  ErrorCovarianceMatrix error_covariance_;
};

}  // namespace hyped::navigation
