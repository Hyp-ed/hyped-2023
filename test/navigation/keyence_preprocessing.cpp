#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocessing/preprocess_keyence.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {
TEST(keyence, equal_data)
{
  utils::ManualTime manual_time;
  core::Logger logger("test", core::LogLevel::kFatal, manual_time);
  navigation::KeyencePreprocessor keyence_processer(logger);
  const core::KeyenceData data = {1, 1};
  const auto final_data        = keyence_processer.checkKeyenceAgrees(data);
  ASSERT_EQ(final_data, navigation::SensorChecks::kAcceptable);
}

TEST(keyence, multi_unequal_data){
    utils::ManualTime manual_time;
    core::Logger logger("test", core::LogLevel::kFatal, manual_time);
    navigation::KeyencePreprocessor keyence_processer(logger);
    const core::KeyenceData data = {1, 2};
    auto final_data        = keyence_processer.checkKeyenceAgrees(data);
    ASSERT_EQ(final_data, navigation::SensorChecks::kAcceptable);
    final_data        = keyence_processer.checkKeyenceAgrees(data);
    ASSERT_EQ(final_data, navigation::SensorChecks::kUnacceptable);
}
}