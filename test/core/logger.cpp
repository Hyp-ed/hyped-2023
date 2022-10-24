#include <gtest/gtest.h>

#include <utils/manual_time.hpp>
#include <core/logger.hpp>

namespace hyped::test {

void test_time(utils::ManualTime &manual_time, const std::time_t time)
{
  const auto time_point = std::chrono::system_clock::from_time_t(time);
  manual_time.set_time(time_point);
  ASSERT_EQ(manual_time.now(), time_point);
}

void logHelper(core::LogLevel level, utils::ManualTime &manual_time, std::string expected_output, bool stdout)
{
  if (stdout) {
    testing::internal::CaptureStdout();
  } else {
    testing::internal::CaptureStderr();
  }
  core::Logger logger("test", level, manual_time);
  logger.log(level, "test");
  if (stdout) {
    ASSERT_EQ(testing::internal::GetCapturedStdout(), expected_output);
  } else {
    ASSERT_EQ(testing::internal::GetCapturedStderr(), expected_output);
  }
}

TEST(Logger, DebugStdout)
{
  utils::ManualTime manual_time;
  logHelper(core::LogLevel::kDebug, manual_time, "01:00:00.000 DEBUG[test] test\n", true);
};

TEST(Logger, InfoStdout)
{
  utils::ManualTime manual_time;
  logHelper(core::LogLevel::kInfo, manual_time, "01:00:00.000 INFO[test] test\n", true);
};

TEST(Logger, FatalStdout)
{
  utils::ManualTime manual_time;
  logHelper(core::LogLevel::kFatal, manual_time, "", true);
};

TEST(Logger, DebugStderr)
{
  utils::ManualTime manual_time;
  logHelper(core::LogLevel::kDebug, manual_time, "", false);
};

TEST(Logger, InfoStderr)
{
  utils::ManualTime manual_time;
  logHelper(core::LogLevel::kInfo, manual_time, "", false);
};

TEST(Logger, FatalStderr)
{
  utils::ManualTime manual_time;
  logHelper(core::LogLevel::kFatal, manual_time, "01:00:00.000 FATAL[test] test\n", false);
};
}  // namespace hyped::test
