#include <gtest/gtest.h>

#include <utils/manual_time.hpp>

namespace hyped::test {

void test_time(utils::ManualTime &manual_time, const std::time_t time)
{
  const auto time_point = std::chrono::high_resolution_clock::from_time_t(time);
  manual_time.set_time(time_point);
  ASSERT_EQ(manual_time.now(), time_point);
}

TEST(ManualTime, basic)
{
  utils::ManualTime manual_time;
  test_time(manual_time, 0);
  test_time(manual_time, 1000);
  test_time(manual_time, 100);
  test_time(manual_time, 2000);
}

}  // namespace hyped::test
