#include <gtest/gtest.h>

#include <navigation/kalman_filter.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

TEST(KalmanFilter, construction)
{
  using KalmanFilter     = navigation::KalmanFilter<3, 3>;
  const auto manual_time = std::make_shared<utils::ManualTime>();
  KalmanFilter::StateVector initial_state;
  KalmanFilter::ErrorCovarianceMatrix initial_error_covariance;
  KalmanFilter kalman_filter(manual_time, initial_state, initial_error_covariance);
  kalman_filter.filter(KalmanFilter::StateTransitionMatrix(),
                       KalmanFilter::StateTransitionCovarianceMatrix(),
                       KalmanFilter::MeasurementMatrix(),
                       KalmanFilter::MeasurementNoiseCovarianceMatrix(),
                       KalmanFilter::StateVector());
}

}  // namespace hyped::test
