#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocess_imu.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

void test()
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
}

TEST(Imu, equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  core::RawImuData data = {{1, 1, 1}};
  core::ImuData answer  = {std::sqrt(3), std::sqrt(3), std::sqrt(3), std::sqrt(3)};
  std::optional<std::array<float, 4>> final_data = imu_processer.processData(data);
  ASSERT_EQ(*final_data, answer);
}

TEST(Imu, not_equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  core::RawImuData data                          = {{1, 1, 1}};
  data.at(0)                                     = {3, 5, 6};
  core::ImuData answer                           = {std::sqrt(3)};
  std::optional<std::array<float, 4>> final_data = imu_processer.processData(data);
  ASSERT_EQ(*final_data, answer);
}

TEST(Imu, one_unreliable_sensor)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  core::RawImuData data = {{1, 1, 1}};
  data.at(0)            = {3, 5, 6};
  for (size_t i; i < 30; ++i) {
    imu_processer.processData(data);
  }
  core::ImuData answer                           = {std::sqrt(3)};
  std::optional<std::array<float, 4>> final_data = imu_processer.processData(data);
  std::array<bool, core::kNumImus> bool_array    = {false, true, true, true};
  ASSERT_EQ(*final_data, answer);
  ASSERT_EQ(imu_processer.getReliablityArray(), bool_array);
}
}  // namespace hyped::test