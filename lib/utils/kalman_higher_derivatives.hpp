#pragma once

#include <cstdint>
#include <vector>

#include <core/logger.hpp>
#include <core/time.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <navigation/consts.hpp>

namespace hyped::utils {
class KalmanHigherDerivatives {
 public:
  KalmanHigherDerivatives(core::ILogger &logger, const core::ITimeSource &time);

  void update(const std::uint64_t timestamp, const core::Float acceleration);

  navigation::JerkSnap getJerkAndSnap();

 private:
  core::ILogger &logger_;
  const core::ITimeSource &time_;

  // TODO: decide 50 or 100 - numerical tests?
  static constexpr std::uint8_t kNumFirstDerivativeValues  = 50;
  static constexpr std::uint8_t kNumSecondDerivativeValues = 10;
  std::vector<core::Float> acceleration_values_;
  std::vector<std::uint64_t> timestamps_;
};
}  // namespace hyped::utils