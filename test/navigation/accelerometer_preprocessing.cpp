#include <iostream>

#include <gtest/gtest.h>

#include "navigation/types.hpp"
#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocess_accelerometer.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

core::Float epsilon = 1e-5;
bool checkArrayEquality(const std::array<core::Float, core::kNumAccelerometers> &data_a,
                        const std::array<core::Float, core::kNumAccelerometers> &data_b)
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
  const core::CombinedRawAccelerometerData data(manual_time.now(),
                                                {{{.x = 1, .y = 0, .z = 0},
                                                  {.x = 1, .y = 0, .z = 0},
                                                  {.x = 1, .y = 0, .z = 0},
                                                  {.x = 1, .y = 0, .z = 0}}});
  const navigation::AccelerometerData expected(manual_time.now(), {1.0, 1.0, 1.0, 1.0});
  const auto calculated = accelerometer_processer.processData(data);
  ASSERT_EQ(calculated->measured_at, expected.measured_at);
  ASSERT_TRUE(checkArrayEquality(calculated->value, expected.value));
}

TEST(Accelerometer, not_equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::AccelerometerPreprocessor accelerometer_processer(logger);
  const core::CombinedRawAccelerometerData data(manual_time.now(),
                                                {{{.x = 3, .y = 5, .z = 6},
                                                  {.x = 1, .y = 0, .z = 0},
                                                  {.x = 1, .y = 0, .z = 0},
                                                  {.x = 1, .y = 0, .z = 0}}});
  const navigation::AccelerometerData expected(manual_time.now(), {1.0, 1.0, 1.0, 1.0});
  const auto calculated = accelerometer_processer.processData(data);
  ASSERT_EQ(calculated->measured_at, expected.measured_at);
  ASSERT_TRUE(checkArrayEquality(calculated->value, expected.value));
}

TEST(Accelerometer, one_unreliable_sensor)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::AccelerometerPreprocessor accelerometer_processer(logger);
  const navigation::AccelerometerData expected(manual_time.now(), {1.0, 1.0, 1.0, 1.0});
  const std::array<core::RawAcceleration, core::kNumAccelerometers> data
    = {{{.x = 3, .y = 5, .z = 6},
        {.x = 1, .y = 0, .z = 0},
        {.x = 1, .y = 0, .z = 0},
        {.x = 1, .y = 0, .z = 0}}};
  for (std::size_t i = 0; i < 100; ++i) {
    manual_time.addSeconds(1);
    const auto calculated = accelerometer_processer.processData(
      core::CombinedRawAccelerometerData(manual_time.now(), data));
    ASSERT_EQ(calculated->measured_at, expected.measured_at);
    ASSERT_TRUE(checkArrayEquality(calculated->value, expected.value));
  }
}

}  // namespace hyped::test
