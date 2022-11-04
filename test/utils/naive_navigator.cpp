#include <gtest/gtest.h>

#include <core/types.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

void testWithTrajectory(utils::NaiveNavigator &naive_navigator, const core::Trajectory &trajectory)
{
  naive_navigator.update(trajectory);
  const core::Trajectory &current_trajectory = naive_navigator.currentTrajectory();
  ASSERT_FLOAT_EQ(current_trajectory.acceleration, trajectory.acceleration);
  ASSERT_FLOAT_EQ(current_trajectory.velocity, trajectory.velocity);
  ASSERT_FLOAT_EQ(current_trajectory.displacement, trajectory.displacement);
}

TEST(NaiveNavigator, basic)
{
  utils::NaiveNavigator naive_navigator;
  testWithTrajectory(naive_navigator, {100, 10, 1});
  testWithTrajectory(naive_navigator, {110, 11, 2});
  testWithTrajectory(naive_navigator, {121, 13, -4});
  testWithTrajectory(naive_navigator, {134, 9, -10});
}

}  // namespace hyped::test
