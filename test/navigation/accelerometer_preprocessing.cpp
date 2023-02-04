#include <iostream>

#include <gtest/gtest.h>

#include "navigation/types.hpp"
#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocess_accelerometer.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

core::Float epsilon = 1e-5;
bool checkArrayEquality(const navigation::AccelerometerData &data_a,
                        const navigation::AccelerometerData &data_b)
{
  if (data_a.size() != data_b.size()) { return false; }
  for (std::size_t i; i < data_a.size(); ++i) {
    if (!(std::abs(data_a.at(i) - data_b.at(i)) < epsilon)) { return false; }
  }
  return true;
}

TEST(Accelerometer, equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::AccelerometerPreprocessor accelerometer_processer(logger);
  const core::RawAccelerometerData data        = {{{1, 1, 1}}};
  const navigation::AccelerometerData expected = {static_cast<core::Float>(std::sqrt(3.0))};
  const auto calculated                        = accelerometer_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*calculated, expected));
}

TEST(Accelerometer, not_equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::AccelerometerPreprocessor accelerometer_processer(logger);
  const core::RawAccelerometerData data        = {{{3, 5, 6}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
  const navigation::AccelerometerData expected = {static_cast<core::Float>(std::sqrt(3.0))};
  const auto calculated                        = accelerometer_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*calculated, expected));
}

TEST(Accelerometer, one_unreliable_sensor)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::AccelerometerPreprocessor accelerometer_processer(logger);
  const core::RawAccelerometerData data = {{{3, 5, 6}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
  for (std::size_t i = 0; i < 22; ++i) {
    accelerometer_processer.processData(data);
  }
  const navigation::AccelerometerData answer = {static_cast<core::Float>(std::sqrt(3.0))};
  const auto final_data                      = accelerometer_processer.processData(data);
  ASSERT_TRUE(checkArrayEquality(*final_data, answer));
}
}  // namespace hyped::test
