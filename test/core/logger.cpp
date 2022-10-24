#include <gtest/gtest.h>

#include <utils/manual_time.hpp>
#include <core/logger.hpp>

namespace hyped::test {

void testStdoutLog(const core::LogLevel level, const utils::ManualTime &manual_time, const std::string expected_output)
{
  testing::internal::CaptureStdout();
  core::Logger logger("test", level, manual_time);
  logger.log(level, "test");
  ASSERT_EQ(testing::internal::GetCapturedStdout(), expected_output);
}

void testStderrLog(const core::LogLevel level, const utils::ManualTime &manual_time, const std::string expected_output)
{
  testing::internal::CaptureStderr();
  core::Logger logger("test", level, manual_time);
  logger.log(level, "test");
  ASSERT_EQ(testing::internal::GetCapturedStderr(), expected_output);
} 

TEST(Logger, Stdout)
{
  utils::ManualTime manual_time;
  testStdoutLog(core::LogLevel::kDebug, manual_time, "01:00:00.000 DEBUG[test] test\n");
  testStdoutLog(core::LogLevel::kInfo, manual_time, "01:00:00.000 INFO[test] test\n");
  testStdoutLog(core::LogLevel::kFatal, manual_time, "");
};


TEST(Logger, Stderr)
{
  utils::ManualTime manual_time;
  testStderrLog(core::LogLevel::kDebug, manual_time, "");
  testStderrLog(core::LogLevel::kInfo, manual_time, "");
  testStderrLog(core::LogLevel::kFatal, manual_time, "01:00:00.000 FATAL[test] test\n");
};
}  // namespace hyped::test
