#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <Eigen/Dense>
#include <core/time.hpp>
#include <core/types.hpp>

namespace hyped::navigation {

template<std::size_t state_dimension, std::size_t measurement_dimension>
class KalmanFilter {
 public:
  using StateVector           = Eigen::Matrix<core::Float, state_dimension, 1>;
  using StateTransitionMatrix = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
  using StateTransitionCovarianceMatrix
    = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
  using ErrorCovarianceMatrix = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
  using MeasurementMatrix     = Eigen::Matrix<core::Float, measurement_dimension, state_dimension>;
  using MeasurementVector     = Eigen::Matrix<core::Float, measurement_dimension, 1>;
  using MeasurementNoiseCovarianceMatrix
    = Eigen::Matrix<core::Float, measurement_dimension, measurement_dimension>;

  KalmanFilter(
    std::shared_ptr<core::ITimeSource> time_source,
    const StateVector initial_state,
    const ErrorCovarianceMatrix initial_error_covariance,
    const std::function<StateTransitionMatrix(const core::Duration dt)> transition_matrix_by_time)
      : time_source_(time_source),
        last_update_time_(time_source->now()),
        state_vector_(initial_state),
        error_covariance_(initial_error_covariance),
        transition_matrix_by_time_(transition_matrix_by_time)
  {
    static_assert(state_dimension > 0);
    static_assert(measurement_dimension > 0);
  }

  void filter(const MeasurementVector &measurement)
  {
    const auto current_update_time = time_source_->now();
    const auto dt                  = current_update_time - last_update_time_;
  }

  const StateVector &getStateEstimate() const { return state_vector_; }
  const ErrorCovarianceMatrix &getErrorCovariance() const { return error_covariance_; }

 private:
  std::shared_ptr<core::ITimeSource> time_source_;
  core::TimePoint last_update_time_;
  StateVector state_vector_;
  ErrorCovarianceMatrix error_covariance_;
  std::function<StateTransitionMatrix(const core::Duration dt)> transition_matrix_by_time_;
};

}  // namespace hyped::navigation
