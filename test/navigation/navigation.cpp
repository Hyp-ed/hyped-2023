#include <gtest/gtest.h>

#include <navigation/consts.hpp>

namespace hyped::test {

TEST(Trajectory, equality)
{
  navigation::Trajectory reference = {100.0, 10.0, 1.0};
  {
    navigation::Trajectory other = {100.0, 10.0, 1.0};
    ASSERT_EQ(reference, other);
  }
  {
    navigation::Trajectory other = {200.0, 10.0, 1.0};
    ASSERT_NE(reference, other);
  }
  {
    navigation::Trajectory other = {100.0, 20.0, 1.0};
    ASSERT_NE(reference, other);
  }
  {
    navigation::Trajectory other = {100.0, 10.0, 2.0};
    ASSERT_NE(reference, other);
  }
}

}  // namespace hyped::test
