#include <gtest/gtest.h>

#include <core/wall_clock.hpp>
#include <navigation/benchmark/benchmark.hpp>
#include <navigation/benchmark/data.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

constexpr std::uint64_t kNanosPerSecond = 1'000'000'000;

navigation::benchmark::Data makeEmptyData()
{
  navigation::benchmark::DataBuilder builder;
  return builder.build();
}

TEST(Benchmark, construction)
{
  core::WallClock wall_clock;
  navigation::benchmark::Benchmark benchmark(wall_clock, makeEmptyData());
  utils::ManualTime manual_time;
  utils::NaiveNavigator naive_navigator(manual_time);
  benchmark.run(manual_time, naive_navigator);
}

navigation::benchmark::Data makeDataFromConstantAcceleration(const core::Float acceleration,
                                                             const std::size_t num_steps)
{
  navigation::benchmark::DataBuilder builder;
  for (std::size_t t = 1; t <= num_steps; ++t) {
    const core::Trajectory trajectory
      = {0.5f * acceleration * acceleration * t * t, acceleration * t, acceleration};
    builder.addUniformAccelerationData(t * kNanosPerSecond, {trajectory.acceleration, 0.0, 0.0});
    builder.addUniformEncoderData(t * kNanosPerSecond,
                                  static_cast<std::uint32_t>(trajectory.displacement));
    builder.addTrajectoryData(t * kNanosPerSecond, trajectory);
  }
  return builder.build();
}

TEST(Benchmark, naiveNavigation)
{
  core::WallClock wall_clock;
  const auto data = makeDataFromConstantAcceleration(1.0, 10);
  navigation::benchmark::Benchmark benchmark(wall_clock, data);
  utils::ManualTime manual_time;
  utils::NaiveNavigator naive_navigator(manual_time);
  const auto result = benchmark.run(manual_time, naive_navigator);
}

}  // namespace hyped::test
