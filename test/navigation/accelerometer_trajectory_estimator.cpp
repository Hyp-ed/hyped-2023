#include <iostream>

#include <gtest/gtest.h>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <navigation/preprocessing/accelerometer_trajectory.hpp>
#include <utils/manual_time.hpp>

namespace hyped::test {
TEST(trajectory, zero_acceleration)
{
  utils::ManualTime manual_time;
  navigation::AccelerometerTrajectoryEstimator accelerometer_trajectory_estimator(manual_time);
  const core::TimePoint t1 = manual_time.now();

  manual_time.set_time(t1 + std::chrono::seconds(2));
  const core::TimePoint t2 = manual_time.now();
  accelerometer_trajectory_estimator.update(0, t2);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 0);
}

TEST(trajectory, constant_acceleration)
{
  utils::ManualTime manual_time;
  navigation::AccelerometerTrajectoryEstimator accelerometer_trajectory_estimator(manual_time);
  const core::TimePoint t1 = manual_time.now();

  manual_time.set_time(t1 + std::chrono::seconds(2));
  const core::TimePoint t2 = manual_time.now();
  accelerometer_trajectory_estimator.update(1.0, t2);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 2.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 2.0);

  manual_time.set_time(t2 + std::chrono::seconds(2));
  const core::TimePoint t3 = manual_time.now();
  accelerometer_trajectory_estimator.update(1.0, t3);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 8.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 4.0);

  manual_time.set_time(t3 + std::chrono::seconds(2));
  const core::TimePoint t4 = manual_time.now();
  accelerometer_trajectory_estimator.update(1.0, t4);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 18.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 6.0);
}
TEST(trajectory, non_constant_acceleration)
{
  utils::ManualTime manual_time;
  navigation::AccelerometerTrajectoryEstimator accelerometer_trajectory_estimator(manual_time);
  const core::TimePoint t1 = manual_time.now();

  manual_time.set_time(t1 + std::chrono::seconds(2));
  const core::TimePoint t2 = manual_time.now();
  accelerometer_trajectory_estimator.update(1.0, t2);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 2.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 2.0);

  manual_time.set_time(t2 + std::chrono::seconds(2));
  const core::TimePoint t3 = manual_time.now();
  accelerometer_trajectory_estimator.update(2.0, t3);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 10.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 6.0);

  manual_time.set_time(t3 + std::chrono::seconds(2));
  const core::TimePoint t4 = manual_time.now();
  accelerometer_trajectory_estimator.update(3.0, t4);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), 28.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), 12.0);
}
TEST(trajectory, negative_acceleration)
{
  utils::ManualTime manual_time;
  navigation::AccelerometerTrajectoryEstimator accelerometer_trajectory_estimator(manual_time);
  const core::TimePoint t1 = manual_time.now();

  manual_time.set_time(t1 + std::chrono::seconds(2));
  const core::TimePoint t2 = manual_time.now();
  accelerometer_trajectory_estimator.update(-1.0, t2);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), -2.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), -2.0);

  manual_time.set_time(t2 + std::chrono::seconds(2));
  const core::TimePoint t3 = manual_time.now();
  accelerometer_trajectory_estimator.update(-2.0, t3);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), -10.0);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), -6.0);

  manual_time.set_time(t3 + std::chrono::seconds(2));
  const core::TimePoint t4 = manual_time.now();
  accelerometer_trajectory_estimator.update(-3.0, t4);
  ASSERT_EQ(accelerometer_trajectory_estimator.getDisplacementEstimate(), -28);
  ASSERT_EQ(accelerometer_trajectory_estimator.getVelocityEstimate(), -12);
}
}  // namespace hyped::test
