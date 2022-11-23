#include <gtest/gtest.h>

#include <core/wall_clock.hpp>
#include <navigation/benchmark/benchmark.hpp>
#include <navigation/benchmark/data.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

navigation::benchmark::Data emptyData()
{
  navigation::benchmark::DataBuilder builder;
  return builder.build();
}

TEST(Benchmark, construction)
{
  core::WallClock wall_clock;
  navigation::benchmark::Benchmark benchmark(wall_clock, emptyData());
  utils::ManualTime manual_time;
  utils::NaiveNavigator naive_navigator(manual_time);
  benchmark.run(manual_time, naive_navigator);
}

TEST(Benchmark, naiveNavigation)
{
  core::WallClock wall_clock;
  navigation::benchmark::Benchmark benchmark(wall_clock, emptyData());
  utils::ManualTime manual_time;
  utils::NaiveNavigator naive_navigator(manual_time);
  const auto result = benchmark.run(manual_time, naive_navigator);
}

}  // namespace hyped::test
