#include <gtest/gtest.h>

#include <utils/manual_time.hpp>
#include <core/logger.hpp>

namespace hyped::test {

void testLog(const core::LogLevel level, const utils::ManualTime &manual_time, const std::string expected_output, const bool use_stdout)
{
  if (use_stdout) {
    testing::internal::CaptureStdout();
  } else {
    testing::internal::CaptureStderr();
  }
  core::Logger logger("test", level, manual_time);
  logger.log(level, "test");
  if (use_stdout) {
    ASSERT_EQ(testing::internal::GetCapturedStdout(), expected_output);
  } else {
    ASSERT_EQ(testing::internal::GetCapturedStderr(), expected_output);
  }
}

TEST(Logger, Stdout)
{
  utils::ManualTime manual_time;
  testLog(core::LogLevel::kDebug, manual_time, "01:00:00.000 DEBUG[test] test\n", true);
  testLog(core::LogLevel::kInfo, manual_time, "01:00:00.000 INFO[test] test\n", true);
  testLog(core::LogLevel::kFatal, manual_time, "", true);
};


TEST(Logger, Stderr)
{
  utils::ManualTime manual_time;
  testLog(core::LogLevel::kDebug, manual_time, "", false);
  testLog(core::LogLevel::kInfo, manual_time, "", false);
  testLog(core::LogLevel::kFatal, manual_time, "01:00:00.000 FATAL[test] test\n", false);
};
}  // namespace hyped::test
