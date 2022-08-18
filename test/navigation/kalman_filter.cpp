#include <gtest/gtest.h>

#include <helper/manual_time.hpp>
#include <navigation/kalman_filter.hpp>

namespace hyped::test {

TEST(KalmanFilter, construction)
{
  const auto manual_time = std::make_shared<helper::ManualTime>();
  navigation::KalmanFilter<3, 3>::StateVector initial_state;
  navigation::KalmanFilter<3, 3>::ErrorCovarianceMatrix initial_error_covariance;
  navigation::KalmanFilter<3, 3> kalman_filter(
    manual_time, initial_state, initial_error_covariance, [](const core::Duration dt) {
      return navigation::KalmanFilter<3, 3>::StateTransitionMatrix();
    });
}

}  // namespace hyped::test
