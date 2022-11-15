#include <gtest/gtest.h>

#include <utils/manual_time.hpp>

namespace hyped::test {

void testSetTime(utils::ManualTime &manual_time, const std::time_t time)
{
  const auto time_point = std::chrono::system_clock::from_time_t(time);
  manual_time.setTime(time_point);
  ASSERT_EQ(manual_time.now(), time_point);
}

TEST(ManualTime, setTime)
{
  utils::ManualTime manual_time;
  testSetTime(manual_time, 0);
  testSetTime(manual_time, 1000);
  testSetTime(manual_time, 100);
  testSetTime(manual_time, 2000);
}

void testSetSecondsSinceEpoch(utils::ManualTime &manual_time,
                                  const std::uint64_t seconds_since_epoch)
{
  manual_time.setSecondsSinceEpoch(seconds_since_epoch);
  ASSERT_EQ(
    std::chrono::duration_cast<std::chrono::seconds>(manual_time.now().time_since_epoch()).count(),
    seconds_since_epoch);
}

TEST(ManualTime, basic)
{
  utils::ManualTime manual_time;
  testSetSecondsSinceEpoch(manual_time, 0);
  testSetSecondsSinceEpoch(manual_time, 1000);
  testSetSecondsSinceEpoch(manual_time, 100);
  testSetSecondsSinceEpoch(manual_time, 2000);
}

}  // namespace hyped::test
