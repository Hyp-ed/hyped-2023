#include <gtest/gtest.h>

#include <core/types.hpp>
#include <navigation/consts.hpp>

namespace hyped::test {

bool checkFlaotEquality(float a, float b)
{
  return ((a - b) < core::kEpsilon) && ((b - a) < core::kEpsilon);
}

void checkTrajectoriesEquality(const core::Trajectory &trajectory1,
                               const core::Trajectory &trajectory2,
                               bool equality)
{
  const bool comparision
    = checkFlaotEquality(trajectory1.acceleration, trajectory2.acceleration)
      && checkFlaotEquality(trajectory1.velocity, trajectory2.velocity)
      && checkFlaotEquality(trajectory1.displacement, trajectory2.displacement);
  ASSERT_EQ(equality, comparision);
}

TEST(Trajectory, equality)
{
  const core::Trajectory reference = {100.0, 10.0, 1.0};
  {
    const core::Trajectory other = {100.0, 10.0, 1.0};
    checkTrajectoriesEquality(reference, other, true);
  }
  {
    const core::Trajectory other = {200.0, 10.0, 1.0};
    checkTrajectoriesEquality(reference, other, false);
  }
  {
    core::Trajectory other = {100.0, 20.0, 1.0};
    checkTrajectoriesEquality(reference, other, false);
  }
  {
    core::Trajectory other = {100.0, 10.0, 2.0};
    checkTrajectoriesEquality(reference, other, false);
  }
}

}  // namespace hyped::test
