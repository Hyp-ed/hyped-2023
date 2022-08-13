#include <gtest/gtest.h>

#include <helper/naive_navigator.hpp>

namespace hyped::test {

void test_with_trajectory(helper::NaiveNavigator &naive_navigator,
                          const navigation::Trajectory &trajectory)
{
  naive_navigator.update(trajectory);
  ASSERT_EQ(naive_navigator.currentTrajectory(), trajectory);
}

TEST(NaiveNavigator, basic)
{
  helper::NaiveNavigator naive_navigator;
  test_with_trajectory(naive_navigator, {100, 10, 1});
  test_with_trajectory(naive_navigator, {110, 11, 2});
  test_with_trajectory(naive_navigator, {121, 13, -4});
  test_with_trajectory(naive_navigator, {134, 9, -10});
}

}  // namespace hyped::test
