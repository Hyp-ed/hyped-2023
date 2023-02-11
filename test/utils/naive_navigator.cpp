#include <numeric>

#include <gtest/gtest.h>

#include <core/types.hpp>
#include <utils/manual_time.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

// TODOLater: improve testing method here!
void testWithTrajectory(
  utils::NaiveNavigator &naive_navigator,
  core::ITimeSource &time_source,
  const std::array<core::RawAcceleration, core::kNumAccelerometers> &acceleration_data,
  const std::array<std::uint32_t, core::kNumEncoders> &encoder_data,
  const std::array<std::uint32_t, core::kNumKeyence> &keyence_data,
  const core::Trajectory &expected_trajectory)
{
  naive_navigator.accelerometerUpdate(
    core::CombinedRawAccelerometerData(time_source.now(), acceleration_data));
  naive_navigator.encoderUpdate(core::RawEncoderData(time_source.now(), encoder_data));
  naive_navigator.keyenceUpdate(core::RawKeyenceData(time_source.now(), keyence_data));
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
  utils::NaiveNavigator naive_navigator;
  manual_time.addSeconds(1);
  testWithTrajectory(naive_navigator,
                     manual_time,
                     {{{.x = 0, .y = 0, .z = 0},
                       {.x = 0, .y = 0, .z = 0},
                       {.x = 0, .y = 0, .z = 0},
                       {.x = 0, .y = 0, .z = 0}}},
                     {0, 0, 0, 0},
                     {0, 0},
                     {0, 0, 0});
  manual_time.addSeconds(1);
  testWithTrajectory(naive_navigator,
                     manual_time,
                     {{{.x = 1, .y = 0, .z = 0},
                       {.x = 1, .y = 0, .z = 0},
                       {.x = 1, .y = 0, .z = 0},
                       {.x = 1, .y = 0, .z = 0}}},
                     {1, 1, 1, 1},
                     {0, 0},
                     {1, 1, 1});
}

}  // namespace hyped::test
