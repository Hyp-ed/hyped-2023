#include <gtest/gtest.h>

#include <core/types.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

void test_with_trajectory(utils::NaiveNavigator &naive_navigator,
                          const core::Trajectory &trajectory)
{
  naive_navigator.update(trajectory);
  ASSERT_EQ(naive_navigator.currentTrajectory(), trajectory);
}

TEST(NaiveNavigator, basic)
{
  utils::NaiveNavigator naive_navigator;
  test_with_trajectory(naive_navigator, {100, 10, 1});
  test_with_trajectory(naive_navigator, {110, 11, 2});
  test_with_trajectory(naive_navigator, {121, 13, -4});
  test_with_trajectory(naive_navigator, {134, 9, -10});
}

}  // namespace hyped::test
