#include "naive_navigator.hpp"

namespace hyped::helper {

NaiveNavigator::NaiveNavigator() : current_trajectory_{}
{
}

navigator::Trajectory NaiveNavigator::currentTrajectory()
{
  return current_trajectory_;
}

void NaiveNavigator::update(const navigator::Trajectory &current_trajectory)
{
  current_trajectory_ = current_trajectory;
}

}  // namespace hyped::helper
