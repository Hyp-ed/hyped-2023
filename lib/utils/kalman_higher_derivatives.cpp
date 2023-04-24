#include "kalman_higher_derivatives.hpp"

#include <algorithm>

namespace hyped::utils {
KalmanHigherDerivatives::KalmanHigherDerivatives(core::ILogger &logger,
                                                 const core::ITimeSource &time)
    : logger_(logger),
      time_(time)
{
  acceleration_values_.resize(50);
  std::fill(acceleration_values_.begin(), acceleration_values_.end(), 0.0F);

  timestamps_.resize(50);
  std::fill(timestamps_.begin(), timestamps_.end(), 0.0F);
}

void KalmanHigherDerivatives::update(const std::uint64_t timestamp, const core::Float acceleration)
{
  acceleration_values_.pop_back();
  acceleration_values_.insert(acceleration_values_.begin(), acceleration);

  timestamps_.pop_back();
  timestamps_.insert(timestamps_.begin(), timestamp);
}

navigation::JerkSnap KalmanHigherDerivatives::getJerkAndSnap()
{
  // TODO: implement.
  navigation::JerkSnap jerk_and_snap;
  return jerk_and_snap;
}

}  // namespace hyped::utils