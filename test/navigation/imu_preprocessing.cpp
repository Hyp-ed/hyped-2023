#include <gtest/gtest.h>
#include <cmath>

#include <core/types.hpp>
#include <navigation/preprocess_imu.hpp>
#include <core/logger.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {

void test(){
    utils::ManualTime manual_time;
    core::Logger logger("test", core::LogLevel::kFatal, manual_time);
    navigation::ImuPreprocessor imu_processer(logger);
}


}  // namespace hyped::test