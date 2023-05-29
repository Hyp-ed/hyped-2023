#include "kalman_higher_derivatives.hpp"

#include <algorithm>

namespace hyped::utils {
KalmanHigherDerivatives::KalmanHigherDerivatives(core::ILogger &logger,
                                                 const core::ITimeSource &time)
    : logger_(logger),
      time_(time),
      sum_acceleration_differences_(0.0F),
      sum_jerk_differences_(0.0F)
{
  acceleration_values_.resize(kNumFirstDerivativeValues);
  std::fill(acceleration_values_.begin(), acceleration_values_.end(), 0.0F);

  timestamps_.resize(kNumFirstDerivativeValues);
  std::fill(timestamps_.begin(), timestamps_.end(), 0);

  acceleration_differences_.resize(kNumFirstDerivativeValues - 1);
  std::fill(acceleration_differences_.begin(), acceleration_differences_.end(), 0.0F);

  jerk_differences_.resize(kNumFirstDerivativeValues - 2);
  std::fill(jerk_differences_.begin(), jerk_differences_.end(), 0.0F);
}

navigation::JerkSnap KalmanHigherDerivatives::getHigherDerivatives(const std::uint64_t timestamp,
                                                                   const core::Float acceleration)
{
  acceleration_values_.pop_back();
  acceleration_values_.insert(acceleration_values_.begin(), acceleration);

  timestamps_.pop_back();
  timestamps_.insert(timestamps_.begin(), timestamp);

  sum_acceleration_differences_
    -= acceleration_differences_.at(acceleration_differences_.size() + 1);
  acceleration_differences_.pop_back();
  acceleration_differences_.insert(acceleration_differences_.begin(),
                                   acceleration_values_.at(0) - acceleration_values_.at(1));
  sum_acceleration_differences_ += acceleration_differences_.at(0);
  core::Float jerk = sum_acceleration_differences_ / kNumFirstDerivativeValues;

  // TODO: finish implementation
}

navigation::JerkSnap KalmanHigherDerivatives::getJerkAndSnap()
{
  // TODO: implement.

  navigation::JerkSnap jerk_and_snap;
  return jerk_and_snap;
}

}  // namespace hyped::utils