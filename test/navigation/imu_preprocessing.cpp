#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocess_imu.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {


bool checkArrayEquality(core::ImuData &imu_data_a, core::ImuData &imu_data_b)
{
  if (imu_data_a.size() == imu_data_b.size()) {
    for (std::size_t i; i < imu_data_a.size(); ++i) {
      if (!(std::abs(imu_data_a.at(i) - imu_data_b.at(i)) < core::kEpsilon)) { return false; }
    }
  } else {
    return false;
  }
  return true;
}

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
  core::RawImuData data                          = {{1, 1, 1}};
  core::ImuData answer                           = {static_cast<core::Float>(std::sqrt(3.0))};
  std::optional<std::array<float, 4>> final_data = imu_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}

TEST(Imu, not_equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  core::RawImuData data                   = {{{3, 5, 6}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
  core::ImuData answer                    = {static_cast<core::Float>(std::sqrt(3.0))};
  std::optional<core::ImuData> final_data = imu_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}

TEST(Imu, one_unreliable_sensor)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  core::RawImuData data = {{{3, 5, 6}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
  for (std::size_t i; i < 22; ++i) {
    imu_processer.processData(data);
  }
  core::ImuData answer                    = {static_cast<core::Float>(std::sqrt(3.0))};
  std::optional<core::ImuData> final_data = imu_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}
}  // namespace hyped::test