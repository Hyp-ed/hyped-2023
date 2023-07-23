#include <cmath>

#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/control/navigator.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

TEST(Navigator, keyence_update_check_1)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::KeyenceData true_value = {1, 1};
  const core::Result result          = navigator.keyenceUpdate({1, 1});
  ASSERT_EQ(result, core::Result::kSuccess);
  const core::KeyenceData updated_value = navigator.getPreviousKeyenceReading();
  ASSERT_EQ(true_value.at(0), updated_value.at(0));
  ASSERT_EQ(true_value.at(1), updated_value.at(1));
}

TEST(Navigator, keyence_update_check_2)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::KeyenceData true_value = {1, 2};
  const core::Result result          = navigator.keyenceUpdate({1, 2});
  ASSERT_EQ(result, core::Result::kSuccess);
  const core::KeyenceData updated_value = navigator.getPreviousKeyenceReading();
  ASSERT_EQ(true_value.at(0), updated_value.at(0));
  ASSERT_EQ(true_value.at(1), updated_value.at(1));
}

TEST(Navigator, keyence_update_check_failure)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::Result result_1 = navigator.keyenceUpdate({1, 2});
  ASSERT_EQ(result_1, core::Result::kSuccess);
  const core::Result result_2 = navigator.keyenceUpdate({1, 3});
  ASSERT_EQ(result_2, core::Result::kFailure);
}

TEST(Navigator, acceleromter_update_1)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::RawAccelerationData one_sensor(1, 1, 1, manual_time.now(), true);
  const core::Result result
    = navigator.accelerometerUpdate({one_sensor, one_sensor, one_sensor, one_sensor});
  ASSERT_EQ(result, core::Result::kSuccess);
  const auto trajectory            = navigator.currentTrajectory();
  const core::Float expected_value = std::sqrt(3) / 20;
  const core::Float tolerance      = 1e-5;
  ASSERT_TRUE(std::abs(trajectory.value().acceleration - expected_value) < tolerance);
}

TEST(Navigator, acceleromter_update_2)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::RawAccelerationData one_sensor(1, 1, 1, manual_time.now(), true);
  core::Result result
    = navigator.accelerometerUpdate({one_sensor, one_sensor, one_sensor, one_sensor});
  ASSERT_EQ(result, core::Result::kSuccess);
  for (std::size_t i = 0; i < 20; ++i) {
    result = navigator.accelerometerUpdate({one_sensor, one_sensor, one_sensor, one_sensor});
  }
  const auto trajectory            = navigator.currentTrajectory();
  const core::Float expected_value = std::sqrt(3);
  const core::Float tolerance      = 1e-5;
  ASSERT_TRUE(std::abs(trajectory.value().acceleration - expected_value) < tolerance);
}

TEST(Navigator, acceleromter_update_check_failure)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::RawAccelerationData normal_value(1, 1, 1, manual_time.now(), true);
  const core::RawAccelerationData outlier_value(10, 10, 10, manual_time.now(), true);
  core::Result result
    = navigator.accelerometerUpdate({normal_value, normal_value, normal_value, normal_value});
  ASSERT_EQ(result, core::Result::kSuccess);
  for (std::size_t i = 0; i < 21; ++i) {
    result
      = navigator.accelerometerUpdate({normal_value, normal_value, normal_value, outlier_value});
  }
  for (std::size_t i = 0; i < 21; ++i) {
    result
      = navigator.accelerometerUpdate({normal_value, normal_value, outlier_value, normal_value});
  }
  ASSERT_EQ(result, core::Result::kFailure);
}

TEST(Navigator, current_trajectory_success)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::RawAccelerationData zero_acc(0, 0, 0, manual_time.now(), true);
  core::Result acceleration_result
    = navigator.accelerometerUpdate({zero_acc, zero_acc, zero_acc, zero_acc});

  core::Result keyence_result = navigator.keyenceUpdate({0, 0});

  const auto trajectory = navigator.currentTrajectory();

  ASSERT_EQ(trajectory.value().acceleration, 0.0);
  ASSERT_EQ(trajectory.value().velocity, 0.0);
  ASSERT_EQ(trajectory.value().displacement, 0.0);
}

TEST(Navigator, current_trajectory_fail)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::Navigator navigator(logger, manual_time);
  const core::RawAccelerationData zero_acc(0, 0, 0, manual_time.now(), true);
  core::Result acceleration_result
    = navigator.accelerometerUpdate({zero_acc, zero_acc, zero_acc, zero_acc});

  core::Result keyence_result = navigator.keyenceUpdate({3, 3});

  const auto trajectory = navigator.currentTrajectory();

  bool fail_flag = false;
  if (!trajectory) { fail_flag = true; }

  ASSERT_TRUE(fail_flag);
}

}  // namespace hyped::test