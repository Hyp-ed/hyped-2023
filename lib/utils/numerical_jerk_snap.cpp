#include "numerical_jerk_snap.hpp"

namespace hyped::utils {
JerkAndSnap::JerkAndSnap(core::ILogger &logger, const core::ITimeSource &time)
{
}

void JerkAndSnap::updateAccelerationValues(const std::uint64_t timestamp,
                                           const core::Float acceleration)
{
  // TODO: implement
}

navigation::JerkSnap JerkAndSnap::getJerkAndSnap()
{
  // TODO: implement.
  navigation::JerkSnap jerk_and_snap;
  return jerk_and_snap;
}

}  // namespace hyped::utils