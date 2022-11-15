#include <gtest/gtest.h>

#include <core/wall_clock.hpp>
#include <navigation/benchmark/benchmark.hpp>
#include <utils/naive_navigator.hpp>

namespace hyped::test {

TEST(Benchmark, construction)
{
  core::WallClock wall_clock;
  navigation::benchmark::Benchmark benchmark(wall_clock, {}, {}, {}, {});
  utils::ManualTime manual_time;
  utils::NaiveNavigator naive_navigator(manual_time);
  benchmark.run(manual_time, naive_navigator);
}

}  // namespace hyped::test
