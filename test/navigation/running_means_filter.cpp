#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/filtering/running_means_filter.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

TEST(RunningMeansFilter, single_new_datapoint)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::RunningMeansFilter running_means_filter(logger, manual_time);
  // works since we always have 20 points used to calculate mean and array is initialised with zeros
  const core::Float calculated_mean = running_means_filter.updateEstimate(20.0);
  const core::Float true_mean       = 1.0;
  ASSERT_EQ(calculated_mean, true_mean);
}

TEST(RunningMeansFilter, all_equal)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::RunningMeansFilter running_means_filter(logger, manual_time);
  core::Float calculated_mean;
  for (std::size_t i = 0; i < 20; ++i) {
    calculated_mean = running_means_filter.updateEstimate(5.0);
  }
  const core::Float true_mean = 5.0;
  ASSERT_EQ(calculated_mean, true_mean);
}

TEST(RunningMeansFilter, two_different_values)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::RunningMeansFilter running_means_filter(logger, manual_time);
  core::Float calculated_mean;
  for (std::size_t i = 0; i < 10; ++i) {
    calculated_mean = running_means_filter.updateEstimate(5.0);
  }
  for (std::size_t i = 0; i < 10; ++i) {
    calculated_mean = running_means_filter.updateEstimate(10.0);
  }
  const core::Float true_mean = 7.5;
  ASSERT_EQ(calculated_mean, true_mean);
}

TEST(RunningMeansFilter, more_than_twenty_updates)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::RunningMeansFilter running_means_filter(logger, manual_time);
  core::Float calculated_mean;
  for (std::size_t i = 0; i < 20; ++i) {
    calculated_mean = running_means_filter.updateEstimate(-1.0);
  }
  for (std::size_t i = 0; i < 5; ++i) {
    calculated_mean = running_means_filter.updateEstimate(3.0);
  }
  const core::Float true_mean = 0.0;
  ASSERT_EQ(calculated_mean, true_mean);
}

}  // namespace hyped::test