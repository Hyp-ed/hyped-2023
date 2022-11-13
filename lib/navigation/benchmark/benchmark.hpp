#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <core/time.hpp>
#include <navigation/navigator.hpp>
#include <utils/manual_time.hpp>

namespace hyped::navigation::benchmark {

struct Result {
  const core::Duration time_taken_to_construct;
  // TODO: more results
};

class Benchmark {
 public:
  using NavigatorBuilder = std::function<INavigator(const core::ITimeSource &)>;

  Benchmark(const core::ITimeSource &time_source,
            const std::vector<NavigatorBuilder> &navigator_builders);

  std::optional<Result> run(
    std::iterator<std::input_iterator_tag, core::EncoderData> encoder_data,
    std::iterator<std::input_iterator_tag, core::KeyenceData> keyence_data,
    std::iterator<std::input_iterator_tag, core::ImuData> imu_data,
    std::iterator<std::input_iterator_tag, core::Trajectory> trajectory_data)
  {
    // TODO: implement
    return std::nullopt;
  }
};

}  // namespace hyped::navigation::benchmark
