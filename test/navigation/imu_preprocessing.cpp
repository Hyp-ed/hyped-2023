#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocess_imu.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

core::Float epsilon = 1e-5;
bool checkArrayEquality(const core::ImuData &imu_data_a, const core::ImuData &imu_data_b)
{
  if (imu_data_a.size() == imu_data_b.size()) {
    for (std::size_t i; i < imu_data_a.size(); ++i) {
      if (!(std::abs(imu_data_a.at(i) - imu_data_b.at(i)) < epsilon)) { return false; }
    }
  } else {
    return false;
  }
  return true;
}

TEST(Imu, equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  const core::RawImuData data = {{1, 1, 1}};
  const core::ImuData answer  = {static_cast<core::Float>(std::sqrt(3.0))};
  const auto final_data       = imu_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}

TEST(Imu, not_equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  const core::RawImuData data = {{{3, 5, 6}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
  const core::ImuData answer  = {static_cast<core::Float>(std::sqrt(3.0))};
  const auto final_data       = imu_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}

TEST(Imu, one_unreliable_sensor)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::ImuPreprocessor imu_processer(logger);
  const core::RawImuData data = {{{3, 5, 6}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
  for (std::size_t i; i < 22; ++i) {
    imu_processer.processData(data);
  }
  const core::ImuData answer = {static_cast<core::Float>(std::sqrt(3.0))};
  const auto final_data      = imu_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}
}  // namespace hyped::test