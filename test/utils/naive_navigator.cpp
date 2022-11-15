#include <numeric>

#include <gtest/gtest.h>

#include <core/types.hpp>
#include <utils/manual_time.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

// TODOLater: improve testing method here!
void testWithTrajectory(utils::NaiveNavigator &naive_navigator,
                        const core::RawImuData imu_data,
                        const core::RawEncoderData encoder_data,
                        const core::RawKeyenceData keyence_data,
                        const core::Trajectory expected_trajectory)
{
  naive_navigator.imuUpdate(imu_data);
  naive_navigator.encoderUpdate(encoder_data);
  naive_navigator.keyenceUpdate(keyence_data);

  const auto &current_trajectory = naive_navigator.currentTrajectory();
  if (current_trajectory) {
    ASSERT_FLOAT_EQ(current_trajectory->displacement, expected_trajectory.displacement);
    ASSERT_FLOAT_EQ(current_trajectory->velocity, expected_trajectory.velocity);
    ASSERT_FLOAT_EQ(current_trajectory->acceleration, expected_trajectory.acceleration);
  }
}

TEST(NaiveNavigator, basic)
{
  utils::ManualTime manual_time;
  utils::NaiveNavigator naive_navigator(manual_time);
  manual_time.addTime(static_cast<core::Duration>(core::kSecond));
  testWithTrajectory(naive_navigator,
                     {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}},
                     {0, 0, 0, 0},
                     {0, 0},
                     {0, 0, 0});
  manual_time.addTime(static_cast<core::Duration>(core::kSecond));
  testWithTrajectory(naive_navigator,
                     {{{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}}},
                     {1, 1, 1, 1},
                     {0, 0},
                     {1, 1, 1});
}

}  // namespace hyped::test
